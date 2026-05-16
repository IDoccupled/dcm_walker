import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import pinocchio as pin #type: ignore
import numpy as np  
from tf2_ros import TransformBroadcaster
import time
from geometry_msgs.msg import TransformStamped, Twist
from tf_transformations import euler_from_matrix

from dcm_walker.spline_trajectory import CubicSplineTrajectory
from dcm_walker.foot_step_generator import StepGenerator
from dcm_walker.dcm_planner import DCMPlanner
from dcm_walker.step_commander import StepCommander

'''Step settings'''
STEP_LENGTH : float = 0.05
STEP_WIDTH : float = 0.058
STEP_THETA : float = 10.0
STEP_HEIGHT : float = 0.02
STEP_DURATION : float = 0.5

COM_HEIGHT : float = 0.198

CTL_FREQUENCY : int = 20
CTRL_DT : float = 1.0 / CTL_FREQUENCY

DCM_FREQUENCY : int = 20
DCM_DT : float = 1.0 / DCM_FREQUENCY

TF_FREQUENCY : int = 20
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

LEFT = 0
RIGHT = 1

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

        self.l_pos = self.r_pos = None

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

        self.trajectory_list = []

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
            else:
                if not self.ready_for_next_step:
                    # self.get_logger().info("[cmd_callback]: Waiting for previous step to complete.")
                    return

                self.get_logger().info(
                    f"[cmd_callback]: Update: Current step idx: {self.cur_step_idx}. "
                    f"Received cmd_vel: linear={cmd_vel:.2f}, angular={cmd_rot:.2f}."
                )
                self.step_generator.update(self.cur_step_idx, cmd_vel, cmd_rot)

        self.dcm_planner.compute(self.step_generator.list())

        # with self.state_lock:        
        self.step_commander.command(self.step_generator.list(), self.dcm_planner.com_traj_array, self.dcm_planner.com_vel_array, self.dcm_planner.com_acc_array)
        for cmd in self.step_commander.command_list:
            if cmd.idx < self.cur_step_idx - 1:
                self.trajectory_list.append([])
                continue
            else:
                spline_traj_l = CubicSplineTrajectory(
                    start_pos=cmd.l_cmd_pos_init,
                    mid_1_pos=cmd.l_cmd_pos_1,
                    mid_2_pos=cmd.l_cmd_pos_2,
                    end_pos=cmd.l_cmd_pos_3,
                    start_vel=cmd.l_cmd_vel_init,
                    mid_1_vel=cmd.l_cmd_vel_1,
                    mid_2_vel=cmd.l_cmd_vel_2,
                    end_vel=cmd.l_cmd_vel_3,
                    start_acc=cmd.l_cmd_acc_init,
                    mid_1_acc=cmd.l_cmd_acc_1,
                    mid_2_acc=cmd.l_cmd_acc_2,
                    end_acc=cmd.l_cmd_acc_3,
                    segment_1_duration=SEGMENT_1_DURATION,
                    segment_2_duration=SEGMENT_2_DURATION,
                    total_duration=SPLINE_DURATION
                )
                spline_traj_r = CubicSplineTrajectory(
                    start_pos=cmd.r_cmd_pos_init,
                    mid_1_pos=cmd.r_cmd_pos_1,
                    mid_2_pos=cmd.r_cmd_pos_2,
                    end_pos=cmd.r_cmd_pos_3,
                    start_vel=cmd.r_cmd_vel_init,
                    mid_1_vel=cmd.r_cmd_vel_1,
                    mid_2_vel=cmd.r_cmd_vel_2,
                    end_vel=cmd.r_cmd_vel_3,
                    start_acc=cmd.r_cmd_acc_init,
                    mid_1_acc=cmd.r_cmd_acc_1,
                    mid_2_acc=cmd.r_cmd_acc_2,
                    end_acc=cmd.r_cmd_acc_3,
                    segment_1_duration=SEGMENT_1_DURATION,
                    segment_2_duration=SEGMENT_2_DURATION,
                    total_duration=SPLINE_DURATION
                )
                self.trajectory_list.append((spline_traj_l, spline_traj_r))

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

            if self.cur_step_idx == steps[-1].nStep: # Reset
                self.step_generator.reset()
                self.cur_step_idx = 0
                self.ready_for_next_step = True
                self.trajectory_list = []
                self.get_logger().info("[ctrl_timer_callback]: Walking pattern completed. Resetting step generator.")
                return

            if self.ready_for_next_step:
                self.ready_for_next_step = False
                self.start_time = time.time()
                start_step_idx = self.cur_step_idx

            t_elapsed = time.time() - self.start_time
            if t_elapsed > SPLINE_DURATION:
                t_elapsed = SPLINE_DURATION
            
            l_pos, _, _ = self.trajectory_list[self.cur_step_idx][LEFT].update(t_elapsed)
            r_pos, _, _ = self.trajectory_list[self.cur_step_idx][RIGHT].update(t_elapsed)

            l_trans = np.array([l_pos[0], l_pos[1], l_pos[2] - COM_HEIGHT])
            l_trans = pin.SE3(np.eye(3), l_trans)
            l_rot = pin.utils.rotate('z', l_pos[3])
            l_rot = pin.SE3(l_rot, np.zeros(3))
            r_trans = np.array([r_pos[0], r_pos[1], r_pos[2] - COM_HEIGHT])
            r_trans = pin.SE3(np.eye(3), r_trans)
            r_rot = pin.utils.rotate('z', r_pos[3])
            r_rot = pin.SE3(r_rot, np.zeros(3))

            self.l_pos = l_trans * l_rot
            self.r_pos = r_trans * r_rot

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
            self.broadcastTF(self.br, T_trans * T_rot, 'map', f'step_{step.nStep}')

        if steps[self.cur_step_idx].is_left():
            self.broadcastTF(self.br, pin.SE3.Identity(), f'step_{self.cur_step_idx}', 'left_foot')
            self.broadcastTF(self.br, self.l_pos.inverse(), 'left_foot', 'CoM')
            self.broadcastTF(self.br, self.r_pos, 'CoM', 'right_foot')
        else:
            self.broadcastTF(self.br, pin.SE3.Identity(), f'step_{self.cur_step_idx}', 'right_foot')
            self.broadcastTF(self.br, self.r_pos.inverse(), 'right_foot', 'CoM')
            self.broadcastTF(self.br, self.l_pos, 'CoM', 'left_foot')
   

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