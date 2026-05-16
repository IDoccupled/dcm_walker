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
            else: # for z, theta use 3rd order polynomial
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
    from dcm_walker.step_commander import StepCommand
    step = []
    a1 = np.array([[0.01666571], [0.03395372], [-0.221], [-0.13278918], [-0.21325496], [0.0]])
    a2 = np.array([[0.0054082], [0.01694776], [-0.221], [-0.06891542], [-0.06623896], [0.0]])
    a3 = np.array([[-0.01072711], [0.02160157], [-0.216], [-0.1037904], [0.15899119], [0.0]])
    step.append(a1)
    step.append(a2)
    step.append(a3)
    print("\n-----------------")
    traj2 = CubicSplineTrajectory(
        start_pos = [0, 0, -0.221],
        mid_1_pos = np.array([step[0][0, 0], step[0][1, 0], step[0][2, 0]]),
        mid_2_pos = np.array([step[1][0, 0], step[1][1, 0], step[1][2, 0]]),
        end_pos = np.array([step[2][0, 0], step[2][1, 0], step[2][2, 0]]),
        segment_1_duration = 0.15,
        segment_2_duration = 0.1,
        total_duration = 0.5,
        start_vel = [0, 0, 0],
        mid_1_vel = np.array([step[0][3, 0], step[0][4, 0], step[0][5, 0]]),
        mid_2_vel = np.array([step[1][3, 0], step[1][4, 0], step[1][5, 0]]),
        end_vel = np.array([step[2][3, 0], step[2][4, 0], step[2][5, 0]])
    )
    
    for t in np.arange(0.00, 0.5, 0.05):
        pos, vel, seg = traj2.update(t)
        print(f"t={t:.2f}s: pos={pos.round(3)}, vel={vel.round(3)}, seg={seg}")
    
    print("-----------------\n")