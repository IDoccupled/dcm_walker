import numpy as np

def polynomial_3rd_order(xi: float, xf: float, vi: float, vf: float,
                         ti: float, tf: float) -> np.ndarray:
    x = np.array([xi, xf, vi, vf])
    T = np.array([[1, ti, ti**2, ti**3],
                  [1, tf, tf**2, tf**3],
                  [0, 1, 2*ti, 3*ti**2],
                  [0, 1, 2*tf, 3*tf**2]])
    a = np.linalg.solve(T, x)
    return a

class CubicSplineTrajectory:
    def __init__(self,
                 start_pos: np.array,
                 mid_1_pos: np.array,
                 mid_2_pos: np.array,
                 end_pos: np.array,
                 segment_1_duration: float = 0.25,
                 segment_2_duration: float = 0.15,
                 total_duration: float = 0.5,
                 start_vel: np.array = 0.0,
                 mid_1_vel: np.array = 0.0,
                 mid_2_vel: np.array = 0.0,
                 end_vel: np.array = 0.0):
        """
        3-segment cubic spline:
        segment 1: start -> mid_1
        segment 2: mid_1 -> mid_2
        segment 3: mid_2 -> end
        """
        self.start_pos = np.array(start_pos, dtype=float)
        self.mid_1_pos   = np.array(mid_1_pos, dtype=float)
        self.mid_2_pos   = np.array(mid_2_pos, dtype=float)
        self.end_pos   = np.array(end_pos, dtype=float)

        self.dim = len(self.start_pos)

        def _vec(v):
            v = np.array(v, dtype=float)
            if v.ndim == 0:
                v = np.full(self.dim, v)
            return v

        self.start_vel = _vec(start_vel)
        self.mid_1_vel   = _vec(mid_1_vel)
        self.mid_2_vel   = _vec(mid_2_vel)
        self.end_vel   = _vec(end_vel)

        self.segment_1_duration = segment_1_duration if segment_1_duration is not None else 0.3 * total_duration
        self.segment_2_duration = segment_2_duration if segment_2_duration is not None else 0.2 * total_duration
        self.total_duration = total_duration

        self.t_start = 0.0
        self.t_mid_1   = self.segment_1_duration
        self.t_mid_2   = self.t_mid_1 + self.segment_2_duration
        self.t_end   = self.total_duration

        self.coeffs_segment1 = []
        self.coeffs_segment2 = []
        self.coeffs_segment3 = []
        self._calculate_coefficients()

    def _calculate_coefficients(self):
        """Use local time for each segment."""
        self.coeffs_segment1 = []
        self.coeffs_segment2 = []
        self.coeffs_segment3 = []
        dur1 = self.segment_1_duration
        dur2 = self.segment_2_duration
        dur3 = self.t_end - self.t_mid_2

        for i in range(self.dim):
            # segment 1: local t in [0, dur1]
            c1 = polynomial_3rd_order(
                self.start_pos[i], self.mid_1_pos[i],
                self.start_vel[i], self.mid_1_vel[i],
                0.0, dur1
            )
            self.coeffs_segment1.append(c1)

            # segment 2: local t in [0, dur2]
            c2 = polynomial_3rd_order(
                self.mid_1_pos[i], self.mid_2_pos[i],
                self.mid_1_vel[i], self.mid_2_vel[i],
                0.0, dur2
            )
            self.coeffs_segment2.append(c2)

            # segment 3: local t in [0, dur3]
            c3 = polynomial_3rd_order(
                self.mid_2_pos[i], self.end_pos[i],
                self.mid_2_vel[i], self.end_vel[i],
                0.0, dur3
            )
            self.coeffs_segment3.append(c3)

    def update(self, t: float):
        pos = np.zeros(self.dim)
        vel = np.zeros(self.dim)

        if t <= self.t_mid_1:
            t_rel = t - self.t_start           # in [0, dur1]
            coeffs_list = self.coeffs_segment1
        elif t <= self.t_mid_2:
            t_rel = t - self.t_mid_1           # in [0, dur2]
            coeffs_list = self.coeffs_segment2
        else:
            t_rel = t - self.t_mid_2           # in [0, dur3]
            coeffs_list = self.coeffs_segment3

        for i in range(self.dim):
            a0, a1, a2, a3 = coeffs_list[i]
            pos[i] = a0 + a1*t_rel + a2*t_rel**2 + a3*t_rel**3
            vel[i] = a1 + 2*a2*t_rel + 3*a3*t_rel**2

        segment_idx = 0 if t <= self.t_mid_1 else (1 if t <= self.t_mid_2 else 2)
        return pos, vel, segment_idx

#######################################################################
if __name__ == "__main__":
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