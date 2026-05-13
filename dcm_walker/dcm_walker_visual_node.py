import sys
import os
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import pinocchio as pin
import numpy as np  
from tf2_ros import TransformBroadcaster
import time
from geometry_msgs.msg import TransformStamped, Twist
from tf_transformations import euler_from_matrix

from dcm_walker.spline_trajectory import CubicSplineTrajectory
from dcm_walker.foot_step_generator import StepGenerator
from dcm_walker.dcm_planner import DCMPlanner
from dcm_walker.step_commander import StepCommander

from rclpy.qos import QoSProfile
from rclpy.duration import Duration

'''Step settings'''
STEP_LENGTH : float = 0.05
STEP_WIDTH : float = 0.058
STEP_THETA : float = 10.0
STEP_HEIGHT : float = 0.02
STEP_DURATION : float = 0.5

CTL_FREQUENCY : int = 20
CTRL_DT : float = 1.0 / CTL_FREQUENCY

DCM_FREQUENCY : int = 20
DCM_DT : float = 1.0 / DCM_FREQUENCY

TF_FREQUENCY : int = 5
TF_DT : float = 1.0 / TF_FREQUENCY

'''CoM Settings'''
COM_HEIGHT : float = 0.198
COM_FROM_BASE_LINK : float = 0.024

'''Step segment idx'''
SINGLE_SUPPORT : int = 4
DOUBLE_SUPPORT_START : int = 7
DOUBLE_SUPPORT_END : int = 9
SPLINE_DURATION = STEP_DURATION
SPLINE_DT = CTRL_DT
SEGMENT_1_DURATION = SPLINE_DT * (SINGLE_SUPPORT + 1)
SEGMENT_2_DURATION = SPLINE_DT * (DOUBLE_SUPPORT_START - SINGLE_SUPPORT)

def skewsimetric(w):
    S = np.array([[0, -w[2], w[1]],
                  [w[2], 0, -w[0]],
                  [-w[1], w[0], 0]])
    return S

class DCMWalkerVisual(Node):    
    def __init__(self):
        super().__init__("dcm_walker_visual")
        self.get_logger().info("DCM Walker Visualizer Node has been started.")

        self.br = TransformBroadcaster(self)
        self.step_generator = StepGenerator()
        self.dcm_planner = DCMPlanner()
        self.step_commander = StepCommander()

        self.cb_group = ReentrantCallbackGroup()
        self.state_lock = threading.Lock()

        self.l_cmd = self.r_cmd = None

        self.cur_step_idx = 0

        self.cmd_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10,
            callback_group=self.cb_group,
        )
        self.ready_for_next_step = True

        self.ctrl_timer = self.create_timer(CTRL_DT, self.ctrl_timer_callback, callback_group=self.cb_group)
        self.tf_timer = self.create_timer(TF_DT, self.tf_timer_callback, callback_group=self.cb_group)

        self.start_time = 0.0

        self.cmd_able_to_update = True

    def cmd_callback(self, msg: Twist):
        with self.state_lock:
            if not self.cmd_able_to_update:
                return
            self.cmd_able_to_update = False

            cmd_vel = msg.linear.x
            cmd_rot = msg.angular.z

            if not self.step_generator.inited:
                self.ready_for_next_step = True
                self.get_logger().info("[cmd_callback]: Received first cmd_vel, initializing walking pattern.")
                self.step_generator.init()
                self.step_generator.update(1, cmd_vel, 0)
                steps = list(self.step_generator.list())
            else:
                if not self.ready_for_next_step:
                    # self.get_logger().info("[cmd_callback]: Waiting for previous step to complete.")
                    return

                self.get_logger().info(
                    f"[cmd_callback]: Update: Current step idx: {self.cur_step_idx}. "
                    f"Received cmd_vel: linear={cmd_vel:.2f}, angular={cmd_rot:.2f}."
                )
                self.step_generator.update(self.cur_step_idx, cmd_vel, cmd_rot)
                steps = list(self.step_generator.list())

        self.dcm_planner.compute(steps)
        com_traj, com_vel = self.dcm_planner.com_traj_array, self.dcm_planner.com_vel_array
        with self.state_lock:
            self.step_commander.command(steps, com_traj, com_vel)

    def ctrl_timer_callback(self):
        start_step_idx = None
        completed_step_idx = None

        with self.state_lock:
            self.cmd_able_to_update = True
            if not self.step_generator.inited:
                return

            steps = self.step_generator.list()
            if not steps:
                return

            if self.cur_step_idx == steps[-1].nStep:
                self.step_generator.reset()
                self.cur_step_idx = 0
                self.ready_for_next_step = True
                self.get_logger().info("[ctrl_timer_callback]: Walking pattern completed. Resetting step generator.")
                return

            if self.ready_for_next_step:
                self.ready_for_next_step = False
                self.start_time = time.time()
                start_step_idx = self.cur_step_idx

            if time.time() - self.start_time > SPLINE_DURATION:
                self.ready_for_next_step = True
                self.cur_step_idx += 1
                completed_step_idx = self.cur_step_idx - 1

        if start_step_idx is not None:
            self.get_logger().info(
                f"[ctrl_timer_callback]: Starting step {start_step_idx} at time {self.start_time:.2f}."
            )
        if completed_step_idx is not None:
            self.get_logger().info(f"[ctrl_timer_callback]: Step {completed_step_idx} completed.")

    def tf_timer_callback(self):
        with self.state_lock:
            if not self.step_generator.inited:
                return
            steps = list(self.step_generator.list())

        if not steps:
            return

        theta_integral = 0.0
        for step in steps:
            t = np.array([step.pos[0], step.pos[1], 0.0])
            theta_integral += step.pos[2]
            R = pin.utils.rotate('z', theta_integral)
            T_trans = pin.SE3(np.eye(3), t)
            T_rot = pin.SE3(R, np.zeros(3))
            self.broadcastTF(self.br, T_trans * T_rot, 'map', f'step_{step.nStep}_{step.footSide}')
   

    def broadcastTF(self, br, HT, parent, child):
        T = pin.SE3(HT)
        q = pin.Quaternion(T.rotation)
        q.normalize()

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = T.translation[0]
        t.transform.translation.y = T.translation[1]
        t.transform.translation.z = T.translation[2]

        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w

        br.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)
    node = DCMWalkerVisual()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     node.get_logger().info("User interrupted")
    # except Exception as e:
    #     node.get_logger().error(f"Error occurred: {e}")
    # finally:
    #     try:
    #         node.destroy_node()
    #     except Exception:
    #         pass
    #     try:
    #         if rclpy.ok():
    #             rclpy.shutdown()
    #     except Exception:
    #         pass

if __name__ == '__main__':
    main()