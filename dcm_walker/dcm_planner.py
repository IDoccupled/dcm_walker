import numpy as np
from dcm_walker.foot_step_generator import StepGenerator
import matplotlib.pyplot as plt
from dcm_walker.xr_tools_py import print_tools as pt

class DCMPlanner:
    '''
    DCM Planner for walking pattern generation.
    
    :param frequency: Control frequency in [Hz].
    :type frequency: int
    :param com_height: Center of mass height from support point in [meters].
    :type com_height: float
    :param step_duration: Duration of a single step in [seconds].
    :type step_duration: float
    :param com_from_base_link: Height of COM from robot base link in [meters].
    :type com_from_base_link: float
    '''
    def __init__(self, 
                 frequency: int = 20,
                 com_height: float = 0.198, 
                 step_duration: float = 0.5,
                 com_from_base_link: float = 0.024,
                 ):

        self.com_height = com_height
        self.frequency = frequency
        self.dt = 1.0 / frequency
        self.step_duration = step_duration
        self.com_from_base_link = com_from_base_link
        self.T_c = np.sqrt( self.com_height / 9.81 )
        self.omega_c = np.sqrt(9.81 / self.com_height)
        self.f_c = (1.0 / np.pi) * np.sqrt(9.81 / self.com_height)

        self.dcm_each_step = self.step_duration * self.frequency
        if not self.dcm_each_step.is_integer():
            raise ValueError("step_duration * frequency must be an integer.")
        self.dcm_each_step = int(self.dcm_each_step)

    def compute(self, step_list: list = None) -> np.ndarray:

        self.vrp_array = np.empty((0, 3))

        self.xi_array = np.empty((0, 3))
        self.xi_r_array = np.empty((0, self.dcm_each_step, 3))
        self.dot_xi_r_array = np.empty((0, self.dcm_each_step, 3))

        self.com_traj_array = np.empty((0, self.dcm_each_step, 3))
        self.com_vel_array = np.empty((0, self.dcm_each_step, 3))
        self.com_acc_array = np.empty((0, self.dcm_each_step, 3))
        
        # VRP
        self.vrp_array = np.array([step.pos for step in step_list]).reshape(-1,3)
        self.vrp_array[:,2] = self.com_height
        self.vrp_array[0,0] = (self.vrp_array[0,0] + self.vrp_array[1,0]) / 2
        self.vrp_array[0,1] = (self.vrp_array[0,1] + self.vrp_array[1,1]) / 2
        self.vrp_array[-1,0] = (self.vrp_array[-1,0] + self.vrp_array[-2,0]) / 2
        self.vrp_array[-1,1] = (self.vrp_array[-1,1] + self.vrp_array[-2,1]) / 2
        
        # Catch points
        self.xi_array = self.vrp_array.copy()
        for i in range(len(self.vrp_array)-1, -1, -1):
            if i == 0 or i == len(self.vrp_array)-1:
                continue
            self.xi_array[i] = self.vrp_array[i] \
                        + np.exp( - self.omega_c * self.step_duration ) \
                        * ( self.xi_array[i+1] - self.vrp_array[i] )
        # Waypoints
        for i in range(len(self.xi_array)-1):
            xi_r_array_temp = []
            dot_xi_r_array_temp = []
            for j in range(self.dcm_each_step):
                j += 1
                xi_r_x = self.vrp_array[i,0] \
                        + np.exp(self.omega_c * (j * self.dt - self.step_duration)) \
                        * (self.xi_array[i+1,0] - self.vrp_array[i,0])
                xi_r_y = self.vrp_array[i,1] \
                        + np.exp(self.omega_c * (j * self.dt - self.step_duration)) \
                        * (self.xi_array[i+1,1] - self.vrp_array[i,1])
                if i == len(self.xi_array)-2 and j == self.dcm_each_step:
                    xi_r_x = self.vrp_array[-1,0]
                    xi_r_y = self.vrp_array[-1,1]
                xi_r_array_temp.append([xi_r_x, xi_r_y, self.com_height])
                
                dot_xi_r_x = self.omega_c * (xi_r_x - self.vrp_array[i,0])
                dot_xi_r_y = self.omega_c * (xi_r_y - self.vrp_array[i,1])
                dot_xi_r_array_temp.append([dot_xi_r_x, dot_xi_r_y, 0.0])

            self.dot_xi_r_array = np.concatenate((self.dot_xi_r_array, 
                                              np.array(dot_xi_r_array_temp).reshape(1, self.dcm_each_step, 3)),
                                              axis=0)
            self.xi_r_array = np.concatenate((self.xi_r_array, 
                                              np.array(xi_r_array_temp).reshape(1, self.dcm_each_step, 3)), 
                                              axis=0)
            
        # COM trajectory
        for i in range(len(self.xi_array)-1):
            com_traj_array_temp = []
            com_vel_array_temp = []
            com_acc_array_temp = []
            prev_com = self.vrp_array[0,:].copy() if i == 0 else self.com_traj_array[i-1,-1,:]
            for j in range(self.dcm_each_step):
                com_prev = prev_com if j == 0 else np.array(com_traj_array_temp[-1])
                vel = self.omega_c * (self.xi_r_array[i,j,:] - com_prev)
                acc = self.omega_c * (self.dot_xi_r_array[i,j,:] - vel)
                com_next = com_prev + vel * self.dt
                if i == len(self.xi_array)-2 and j == self.dcm_each_step-1:
                    com_next = self.vrp_array[-1,:].copy()
                    vel = np.zeros(3)
                    acc = np.zeros(3)
                com_vel_array_temp.append(vel.tolist())
                com_acc_array_temp.append(acc.tolist())
                com_traj_array_temp.append(com_next.tolist())
            self.com_traj_array = np.concatenate((self.com_traj_array, 
                                                  np.array(com_traj_array_temp).reshape(1, self.dcm_each_step, 3)),
                                                  axis=0)
            self.com_vel_array = np.concatenate((self.com_vel_array, 
                                                 np.array(com_vel_array_temp).reshape(1, self.dcm_each_step, 3)),
                                                 axis=0)
            self.com_acc_array = np.concatenate((self.com_acc_array, 
                                                 np.array(com_acc_array_temp).reshape(1, self.dcm_each_step, 3)),
                                                 axis=0)

    def visualize(self, 
                  threeD: bool = False,
                  print_log: bool = False,
                  step_list: list = None):

        if print_log:
            print("="*60)
            print("VRP Array, size:", self.vrp_array.shape)
            print(self.vrp_array)
            print("="*60)
            print("DCM Array, size:", self.xi_array.shape)
            print(self.xi_array)
            print("="*60)
            print("DCM Waypoints Array, size:", self.xi_r_array.shape)
            print(self.xi_r_array)
            print("="*60)
            print("COM Trajectory Array, size:", self.com_traj_array.shape)
            print("COM Velocity Array, size:", self.com_vel_array.shape)
            print("COM Acceleration Array, size:", self.com_acc_array.shape)
            for i in range(self.com_traj_array.shape[0]):
                pt.printgreen(f"Step {i}, swing foot {'left' if step_list[i+1].is_left() else 'right'}:")
                for j in range(self.com_traj_array.shape[1]):
                    print(f"Time {j*self.dt:.2f}s: ")
                    print(f"COM Position = ({self.com_traj_array[i,j,0]:.3f}, {self.com_traj_array[i,j,1]:.3f}, {self.com_traj_array[i,j,2]:.3f})")
                    print(f"COM Velocity = ({self.com_vel_array[i,j,0]:.3f}, {self.com_vel_array[i,j,1]:.3f}, {self.com_vel_array[i,j,2]:.3f})")
                    print(f"COM Acceleration = ({self.com_acc_array[i,j,0]:.3f}, {self.com_acc_array[i,j,1]:.3f}, {self.com_acc_array[i,j,2]:.3f})")
            print("="*60)
            for step in step_list:
                pt.printyellow(f'Step {step.nStep} , Foot {"Left" if step.is_left() else "Right"}')
                print(f'Position = ({step.pos[0]:.3f}, {step.pos[1]:.3f}, {step.pos[2]:.3f})')
            print("="*60)   

        plt_3d = threeD
        if plt_3d:
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            ax.plot(self.vrp_array[:,0], 
                    self.vrp_array[:,1], 
                    self.vrp_array[:,2], 
                    'ro', label='VRP')
            ax.plot(self.xi_array[:,0], 
                    self.xi_array[:,1], 
                    self.xi_array[:,2], 
                    'bx', label='DCM')
            for i in range(self.xi_r_array.shape[0]):
                ax.plot(self.xi_r_array[i,:,0], 
                        self.xi_r_array[i,:,1], 
                        self.xi_r_array[i,:,2], 
                        'go-', label='DCM Waypoints' if i == 0 else "")
            for i in range(self.com_traj_array.shape[0]):
                ax.plot(self.com_traj_array[i,:,0], 
                        self.com_traj_array[i,:,1],
                        self.com_traj_array[i,:,2],
                        'r-', label='CoM Trajectory' if i == 0 else "")
        else:
            fig, ax = plt.subplots()

            theta = 0.0
            for step in step_list:
                plt.plot(step.pos[0], step.pos[1], 'ro' if step.is_left() else 'bo')
                plt.text(step.pos[0], step.pos[1], f'{"L" if step.is_left() else "R"}, {step.nStep}', 
                        fontsize=9, ha='center', va='bottom')
                arrow_length = 0.03
                theta += step.pos[2]
                plt.arrow(step.pos[0], step.pos[1], arrow_length * np.cos(theta), 
                        arrow_length * np.sin(theta), 
                        head_width=0.005, head_length=0.003, 
                        fc='gray', ec='gray')
            ax.plot(self.vrp_array[:,0], 
                    self.vrp_array[:,1], 
                    'ro', label='VRP')
            ax.plot(self.xi_array[:,0], 
                    self.xi_array[:,1], 
                    'bx', label='DCM')
            xi_r_flat = self.xi_r_array.reshape(-1, 3)
            ax.plot(xi_r_flat[:,0],
                    xi_r_flat[:,1],
                    'go-', label='DCM Waypoints')
            for i in range(self.com_traj_array.shape[0]):
                ax.plot(self.com_traj_array[i,:,0], 
                        self.com_traj_array[i,:,1],
                        'r.-'if step_list[i+1].is_left() else 'b.-', 
                        label='CoM Trajectory' if i == 0 else "")
            
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        if plt_3d: ax.set_zlabel('Z (m)')
        ax.set_title('DCM Planner Visualization')
        ax.axis('equal')
        ax.legend()
        plt.grid(True)
        plt.show()

def main():
    footstep = StepGenerator()
    footstep.init()
    footstep.update(1, 1, 1)
    footstep.update(2, 1, 1)
    footstep.update(3, 1, 1)
    footstep.update(4, 1, 1)
    footstep.update(5, 1, 1)
    footstep.update(6, 1, 1)
    footstep.update(7, 1, 1)
    footstep.update(8, 1, 1)
    footstep.update(9, 1, 1)
    footstep.update(10, 1, 1)
    # footstep.update(11, 1, 1)
    # footstep.update(12, 1, 1)
    # footstep.update(13, 1, 1)
    step_list = footstep.list()

    planner = DCMPlanner()
    planner.compute(step_list)

    planner.visualize(threeD=False,
                      print_log=True,
                      step_list=step_list)

if __name__ == "__main__":
    main()