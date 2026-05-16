import numpy as np

def polynomial_3rd_order(xi: float, xf: float, 
                         vi: float, vf: float,
                         ti: float, tf: float) -> np.ndarray:
    x = np.array([xi, xf, vi, vf])
    T = np.array([[1, ti, ti**2, ti**3],
                  [1, tf, tf**2, tf**3],
                  [0, 1, 2*ti, 3*ti**2],
                  [0, 1, 2*tf, 3*tf**2]])
    a = np.linalg.solve(T, x)
    return a
def polynomial_3rd_order_eval(a: np.ndarray, t: float) -> (float, float): # type: ignore
    position = a[0] + a[1] * t + a[2] * t**2 + a[3] * t**3
    velocity = a[1] + 2 * a[2] * t + 3 * a[3] * t**2
    return position, velocity

def polynomial_5th_order(xi: float, xf: float, 
                         vi: float, vf: float,
                         ai: float, af: float,
                         ti: float, tf: float) -> np.ndarray:
    x = np.array([xi, xf, vi, vf, ai, af])
    T = np.array([[1, ti, ti**2, ti**3, ti**4, ti**5],
                  [1, tf, tf**2, tf**3, tf**4, tf**5],
                  [0, 1, 2*ti, 3*ti**2, 4*ti**3, 5*ti**4],
                  [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                  [0, 0, 2, 6*ti, 12*ti**2, 20*ti**3],
                  [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])
    a = np.linalg.solve(T, x)
    return a
def polynomial_5th_order_eval(a: np.ndarray, t: float) -> (float, float, float): # type: ignore
    position = a[0] + a[1] * t + a[2] * t**2 + a[3] * t**3 + a[4] * t**4 + a[5] * t**5
    velocity = a[1] + 2 * a[2] * t + 3 * a[3] * t**2 + 4 * a[4] * t**3 + 5 * a[5] * t**4
    acceleration = 2 * a[2] + 6 * a[3] * t + 12 * a[4] * t**2 + 20 * a[5] * t**3
    return position, velocity, acceleration

class CubicSplineTrajectory:
    def __init__(self,
                 start_pos: list = [0, 0, 0, 0],
                 mid_1_pos: list = [0, 0, 0, 0],
                 mid_2_pos: list = [0, 0, 0, 0],
                 end_pos: list = [0, 0, 0, 0],
                 start_vel: list = [0, 0],
                 mid_1_vel: list = [0, 0],
                 mid_2_vel: list = [0, 0],
                 end_vel: list = [0, 0],
                 start_acc: list = [0, 0],
                 mid_1_acc: list = [0, 0],
                 mid_2_acc: list = [0, 0],
                 end_acc: list = [0, 0],
                 segment_1_duration: float = 0.25,
                 segment_2_duration: float = 0.15,
                 total_duration: float = 0.5):
        """
        3-segment cubic spline:
        segment 1: start -> mid_1
        segment 2: mid_1 -> mid_2
        segment 3: mid_2 -> end
        pos: x, y, z, theta
        vel, acc: x, y
        """
        start_pos = np.array(start_pos, dtype=float)
        mid_1_pos = np.array(mid_1_pos, dtype=float)
        mid_2_pos = np.array(mid_2_pos, dtype=float)
        end_pos = np.array(end_pos, dtype=float)

        self.dim = len(start_pos)

        start_vel = np.array(start_vel, dtype=float)
        mid_1_vel = np.array(mid_1_vel, dtype=float)
        mid_2_vel = np.array(mid_2_vel, dtype=float)
        end_vel = np.array(end_vel, dtype=float)

        start_acc = np.array(start_acc, dtype=float)
        mid_1_acc = np.array(mid_1_acc, dtype=float)
        mid_2_acc = np.array(mid_2_acc, dtype=float)
        end_acc = np.array(end_acc, dtype=float)
        
        segment_1_duration = segment_1_duration if segment_1_duration is not None else 0.3 * total_duration
        segment_2_duration = segment_2_duration if segment_2_duration is not None else 0.2 * total_duration

        self.t_mid_1 = segment_1_duration
        self.t_mid_2 = self.t_mid_1 + segment_2_duration
        
        """Use local time for each segment."""
        self.coeffs_segment1 = []
        self.coeffs_segment2 = []
        self.coeffs_segment3 = []
        dur1 = segment_1_duration
        dur2 = segment_2_duration
        dur3 = total_duration - self.t_mid_2

        for i in range(self.dim):
            '''
            i = 0: x
            i = 1: y
            i = 2: z
            i = 3: theta
            '''
            if i <= 1: # for x, y use 5th order polynomial
                c1 = polynomial_5th_order(
                    start_pos[i], mid_1_pos[i],
                    start_vel[i], mid_1_vel[i],
                    start_acc[i], mid_1_acc[i],
                    0.0, dur1
                )
                self.coeffs_segment1.append(c1)

                c2 = polynomial_5th_order(
                    mid_1_pos[i], mid_2_pos[i],
                    mid_1_vel[i], mid_2_vel[i],
                    mid_1_acc[i], mid_2_acc[i],
                    0.0, dur2
                )
                self.coeffs_segment2.append(c2)

                c3 = polynomial_5th_order(
                    mid_2_pos[i], end_pos[i],
                    mid_2_vel[i], end_vel[i],
                    mid_2_acc[i], end_acc[i],
                    0.0, dur3
                )
                self.coeffs_segment3.append(c3)
            elif i == 2:
                c1 = polynomial_3rd_order(
                    start_pos[i], mid_1_pos[i],
                    0.0, 0.0,
                    0.0, dur1
                )
                self.coeffs_segment1.append(c1)
                
                c2 = polynomial_3rd_order(
                    mid_1_pos[i], mid_2_pos[i],
                    0.0, 0.0,
                    0.0, dur2
                )
                self.coeffs_segment2.append(c2)
                
                c3 = polynomial_3rd_order(
                    mid_2_pos[i], end_pos[i],
                    0.0, 0.0,
                    0.0, dur3
                )
                self.coeffs_segment3.append(c3)
            elif i == 3: # TODO: by the end of the final step, dottheta should NOT be zero
                c1 = polynomial_3rd_order(
                    start_pos[i], mid_1_pos[i],
                    0.0, 0.0,
                    0.0, dur1
                )
                self.coeffs_segment1.append(c1)
                
                c2 = polynomial_3rd_order(
                    mid_1_pos[i], mid_2_pos[i],
                    0.0, 0.0,
                    0.0, dur2
                )
                self.coeffs_segment2.append(c2)
                # TODO: to be modified
                c3 = polynomial_3rd_order(
                    mid_2_pos[i], end_pos[i],
                    0.0, 0.0,
                    0.0, dur3
                )
                self.coeffs_segment3.append(c3)

    def update(self, t: float):
        pos = np.zeros(self.dim)
        vel = np.zeros(self.dim)

        if t <= self.t_mid_1:
            t_rel = t                          # in [0, dur1]
            coeffs_list = self.coeffs_segment1
        elif t <= self.t_mid_2:
            t_rel = t - self.t_mid_1           # in [0, dur2]
            coeffs_list = self.coeffs_segment2
        else:
            t_rel = t - self.t_mid_2           # in [0, dur3]
            coeffs_list = self.coeffs_segment3

        for i in range(self.dim):
            if i <= 1: # x, y
                pos[i], vel[i], _ = polynomial_5th_order_eval(coeffs_list[i], t_rel)
            else: # z, theta
                pos[i], vel[i] = polynomial_3rd_order_eval(coeffs_list[i], t_rel)

        segment_idx = 0 if t <= self.t_mid_1 else (1 if t <= self.t_mid_2 else 2)
        return pos, vel, segment_idx

#######################################################################
if __name__ == "__main__":
    from dcm_walker.foot_step_generator import StepGenerator
    from dcm_walker.dcm_planner import DCMPlanner
    from dcm_walker.step_commander import StepCommander

    from matplotlib import pyplot as plt
    
    footstep = StepGenerator()
    planner = DCMPlanner()
    commander = StepCommander()

    footstep.init()
    for i in range(1, 5):
        footstep.update(i, 1 , -1)
    steps = footstep.list()

    planner.compute(steps)
    commander.command(steps, planner.com_traj_array, planner.com_vel_array, planner.com_acc_array)
    cmd_list = commander.command_list

    for cmd in cmd_list:
        print("Command:\n", cmd)

    test_cmd = cmd_list[-4]
    print("Used command:\n", test_cmd)

    spline_traj_l = CubicSplineTrajectory(
        start_pos=test_cmd.l_cmd_pos_init,
        mid_1_pos=test_cmd.l_cmd_pos_1,
        mid_2_pos=test_cmd.l_cmd_pos_2,
        end_pos=test_cmd.l_cmd_pos_3,
        start_vel=test_cmd.l_cmd_vel_init,
        mid_1_vel=test_cmd.l_cmd_vel_1,
        mid_2_vel=test_cmd.l_cmd_vel_2,
        end_vel=test_cmd.l_cmd_vel_3,
        start_acc=test_cmd.l_cmd_acc_init,
        mid_1_acc=test_cmd.l_cmd_acc_1,
        mid_2_acc=test_cmd.l_cmd_acc_2,
        end_acc=test_cmd.l_cmd_acc_3,
        segment_1_duration=0.25,
        segment_2_duration=0.15,
        total_duration=0.5
    )

    freq = 100
    dt = 1.0 / freq
    t = 0.0
    pos_list = []
    for _ in range(int(0.5 / dt) + 1):
        pos, vel, seg_idx = spline_traj_l.update(t)
        print(f"t={t:.2f},\n segment={seg_idx+1},\n pos={pos},\n vel={vel}")
        t += dt
        pos_list.append(pos)

    # Plot 3D trajectory
    pos_array = np.array(pos_list)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos_array[:, 0], pos_array[:, 1], pos_array[:, 2], label='Cubic Spline Trajectory')
    ax.scatter(test_cmd.l_cmd_pos_init[0], test_cmd.l_cmd_pos_init[1], test_cmd.l_cmd_pos_init[2], color='red', label='Start')
    ax.scatter(test_cmd.l_cmd_pos_1[0], test_cmd.l_cmd_pos_1[1], test_cmd.l_cmd_pos_1[2], color='green', label='Mid 1')
    ax.scatter(test_cmd.l_cmd_pos_2[0], test_cmd.l_cmd_pos_2[1], test_cmd.l_cmd_pos_2[2], color='orange', label='Mid 2')
    ax.scatter(test_cmd.l_cmd_pos_3[0], test_cmd.l_cmd_pos_3[1], test_cmd.l_cmd_pos_3[2], color='blue', label='End')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Cubic Spline Trajectory')
    ax.legend()
    plt.show()

