import numpy as np
from dcm_walker.step import Step
from dcm_walker.foot_step_generator import StepGenerator
from dcm_walker.dcm_planner import DCMPlanner
import matplotlib.pyplot as plt
from dcm_walker.xr_tools_py import print_tools as pt

class StepCommand:
    '''
    A class to represent step position command for each step.

    Each command includes three commands for each foot: middle of single support, beginning and end of double support. 
    
    The command is represented as [x, y, lift_height, rot, vel_x, vel_y] in [meters] [radians] and [meters/second].
    
    :param idx: Step index of current support foot.
    :type idx: int
    :param l_cmd_1: Left foot command for single support.
    :type l_cmd_1: list[float]
    :param l_cmd_2: Left foot command for beginning of double support.
    :type l_cmd_2: list[float]
    :param l_cmd_3: Left foot command for end of double support.
    :type l_cmd_3: list[float]
    :param r_cmd_1: Right foot command for single support.
    :type r_cmd_1: list[float]
    :param r_cmd_2: Right foot command for beginning of double support.
    :type r_cmd_2: list[float]
    :param r_cmd_3: Right foot command for end of double support.
    :type r_cmd_3: list[float]
    '''
    def __init__(self, 
                 idx: int = 0,
                 l_cmd_init: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 l_cmd_1:    list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 l_cmd_2:    list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 l_cmd_3:    list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 r_cmd_init: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 r_cmd_1:    list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 r_cmd_2:    list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 r_cmd_3:    list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        self.idx = idx
        self.l_cmd_init = l_cmd_init
        self.l_cmd_1 = l_cmd_1
        self.l_cmd_2 = l_cmd_2
        self.l_cmd_3 = l_cmd_3
        self.r_cmd_init = r_cmd_init
        self.r_cmd_1 = r_cmd_1
        self.r_cmd_2 = r_cmd_2
        self.r_cmd_3 = r_cmd_3

    def __str__(self):
        return f"{pt.textyellow(f"Step idx: {self.idx}")}\n" \
               f"L_cmd   : pos_x, pos_y, lift,   rot,  vel_x, vel_y, acc_x, acc_y\n" \
               f"init    : {float(self.l_cmd_init[0]):.3f}, {float(self.l_cmd_init[1]):.3f}, {float(self.l_cmd_init[2]):.3f}, {float(self.l_cmd_init[3]):.3f}, {float(self.l_cmd_init[4]):.3f}, {float(self.l_cmd_init[5]):.3f}, {float(self.l_cmd_init[6]):.3f}, {float(self.l_cmd_init[7]):.3f}\n" \
               f"ss      : {float(self.l_cmd_1[0]):.3f}, {float(self.l_cmd_1[1]):.3f}, {float(self.l_cmd_1[2]):.3f}, {float(self.l_cmd_1[3]):.3f}, {float(self.l_cmd_1[4]):.3f}, {float(self.l_cmd_1[5]):.3f}, {float(self.l_cmd_1[6]):.3f}, {float(self.l_cmd_1[7]):.3f}\n" \
               f"ds begin: {float(self.l_cmd_2[0]):.3f}, {float(self.l_cmd_2[1]):.3f}, {float(self.l_cmd_2[2]):.3f}, {float(self.l_cmd_2[3]):.3f}, {float(self.l_cmd_2[4]):.3f}, {float(self.l_cmd_2[5]):.3f}, {float(self.l_cmd_2[6]):.3f}, {float(self.l_cmd_2[7]):.3f}\n" \
               f"ds end  : {float(self.l_cmd_3[0]):.3f}, {float(self.l_cmd_3[1]):.3f}, {float(self.l_cmd_3[2]):.3f}, {float(self.l_cmd_3[3]):.3f}, {float(self.l_cmd_3[4]):.3f}, {float(self.l_cmd_3[5]):.3f}, {float(self.l_cmd_3[6]):.3f}, {float(self.l_cmd_3[7]):.3f}\n" \
               f"R_cmd   : pos_x, pos_y, lift,   rot,  vel_x, vel_y, acc_x, acc_y\n" \
               f"init    : {float(self.r_cmd_init[0]):.3f}, {float(self.r_cmd_init[1]):.3f}, {float(self.r_cmd_init[2]):.3f}, {float(self.r_cmd_init[3]):.3f}, {float(self.r_cmd_init[4]):.3f}, {float(self.r_cmd_init[5]):.3f}, {float(self.r_cmd_init[6]):.3f}, {float(self.r_cmd_init[7]):.3f}\n" \
               f"ss      : {float(self.r_cmd_1[0]):.3f}, {float(self.r_cmd_1[1]):.3f}, {float(self.r_cmd_1[2]):.3f}, {float(self.r_cmd_1[3]):.3f}, {float(self.r_cmd_1[4]):.3f}, {float(self.r_cmd_1[5]):.3f}, {float(self.r_cmd_1[6]):.3f}, {float(self.r_cmd_1[7]):.3f}\n" \
               f"ds begin: {float(self.r_cmd_2[0]):.3f}, {float(self.r_cmd_2[1]):.3f}, {float(self.r_cmd_2[2]):.3f}, {float(self.r_cmd_2[3]):.3f}, {float(self.r_cmd_2[4]):.3f}, {float(self.r_cmd_2[5]):.3f}, {float(self.r_cmd_2[6]):.3f}, {float(self.r_cmd_2[7]):.3f}\n" \
               f"ds end  : {float(self.r_cmd_3[0]):.3f}, {float(self.r_cmd_3[1]):.3f}, {float(self.r_cmd_3[2]):.3f}, {float(self.r_cmd_3[3]):.3f}, {float(self.r_cmd_3[4]):.3f}, {float(self.r_cmd_3[5]):.3f}, {float(self.r_cmd_3[6]):.3f}, {float(self.r_cmd_3[7]):.3f}\n"

    def idx(self) -> int:
        return self.idx
    
    @property
    def l_cmd_pos_init(self) -> list[float]:
        return self.l_cmd_init[:4]
    @property
    def l_cmd_vel_init(self) -> list[float]:
        return self.l_cmd_init[4:6]
    @property
    def l_cmd_acc_init(self) -> list[float]:
        return self.l_cmd_init[6:8]
    @property
    def l_cmd_pos_1(self) -> list[float]:
        return self.l_cmd_1[:4]
    @property
    def l_cmd_pos_2(self) -> list[float]:
        return self.l_cmd_2[:4]
    @property
    def l_cmd_pos_3(self) -> list[float]:
        return self.l_cmd_3[:4]
    @property
    def l_cmd_vel_1(self) -> list[float]:
        return self.l_cmd_1[4:6]
    @property
    def l_cmd_vel_2(self) -> list[float]:
        return self.l_cmd_2[4:6]
    @property
    def l_cmd_vel_3(self) -> list[float]:
        return self.l_cmd_3[4:6]
    @property
    def l_cmd_acc_1(self) -> list[float]:
        return self.l_cmd_1[6:8]
    @property
    def l_cmd_acc_2(self) -> list[float]:
        return self.l_cmd_2[6:8]
    @property
    def l_cmd_acc_3(self) -> list[float]:
        return self.l_cmd_3[6:8]

    @property
    def r_cmd_pos_init(self) -> list[float]:
        return self.r_cmd_init[:4]
    @property
    def r_cmd_vel_init(self) -> list[float]:
        return self.r_cmd_init[4:6]
    @property
    def r_cmd_acc_init(self) -> list[float]:
        return self.r_cmd_init[6:8]
    @property
    def r_cmd_pos_1(self) -> list[float]:
        return self.r_cmd_1[:4]
    @property
    def r_cmd_pos_2(self) -> list[float]:
        return self.r_cmd_2[:4]
    @property
    def r_cmd_pos_3(self) -> list[float]:
        return self.r_cmd_3[:4]
    @property
    def r_cmd_vel_1(self) -> list[float]:
        return self.r_cmd_1[4:6]
    @property
    def r_cmd_vel_2(self) -> list[float]:
        return self.r_cmd_2[4:6]
    @property
    def r_cmd_vel_3(self) -> list[float]:
        return self.r_cmd_3[4:6]
    @property
    def r_cmd_acc_1(self) -> list[float]:
        return self.r_cmd_1[6:8]
    @property
    def r_cmd_acc_2(self) -> list[float]:
        return self.r_cmd_2[6:8]
    @property
    def r_cmd_acc_3(self) -> list[float]:
        return self.r_cmd_3[6:8]

    @property
    def l_cmd_last(self) -> list[float]:
        return self.l_cmd_3
    @property
    def r_cmd_last(self) -> list[float]:
        return self.r_cmd_3

class StepCommander:
    def __init__(self,
                 step_height: float = 0.02,
                 ss: int = 4,
                 ds_begin: int = 7,
                 ds_end: int = 9):
        self.step_height = step_height
        self.ss = ss
        self.ds_begin = ds_begin
        self.ds_end = ds_end

        self.__command_list = []

    def _to_com_frame(self,
                      foot_x: float,
                      foot_y: float,
                      com_x: float,
                      com_y: float,
                      com_theta: float) -> tuple[float, float]:
        dx = foot_x - com_x
        dy = foot_y - com_y
        c = np.cos(com_theta)
        s = np.sin(com_theta)
        return dx * c + dy * s, -dx * s + dy * c

    def command(self,
                step_list: list[Step],
                com_traj_array: np.ndarray,
                com_vel_array: np.ndarray,
                com_acc_array: np.ndarray) -> list[StepCommand]:
            
            self.__command_list = []

            if len(step_list) < 2:
                return self.__command_list

            yaw_list = np.cumsum([step.pos[2] for step in step_list])

            for list_idx in range(len(step_list)-1):
                predict_idx = step_list[list_idx].nStep
                if step_list[list_idx].is_left(): # If left foot is current support foot
                    left = list_idx
                    right = list_idx + 1
                    # Only support foot can have rotation command to prevent crossing steps during turning
                    # Only swing foot can have lift command
                    l_theta = step_list[right].pos[2]
                    l_cmd_z = 0
                    r_theta = 0
                    r_cmd_z = self.step_height
                else:
                    left = list_idx + 1
                    right = list_idx
                    l_theta = 0
                    l_cmd_z = self.step_height
                    r_theta = step_list[left].pos[2]
                    r_cmd_z = 0

                com_theta = yaw_list[list_idx + 1]

                if predict_idx == 0:
                    l_cmd_z = 0
                    r_cmd_z = 0

                l_x = step_list[left].pos[0]
                l_y = step_list[left].pos[1]
                r_x = step_list[right].pos[0]
                r_y = step_list[right].pos[1]

                if predict_idx == 0:
                    l_cmd_init = [l_x, l_y, l_cmd_z, l_theta, 0.0, 0.0, 0.0, 0.0]
                    r_cmd_init = [r_x, r_y, r_cmd_z, r_theta, 0.0, 0.0, 0.0, 0.0]
                else:
                    l_cmd_init = self.__command_list[-1].l_cmd_last
                    r_cmd_init = self.__command_list[-1].r_cmd_last

                com_x_1 = com_traj_array[list_idx, self.ss, 0]
                com_y_1 = com_traj_array[list_idx, self.ss, 1]
                com_dotx_1 = com_vel_array[list_idx, self.ss, 0]
                com_doty_1 = com_vel_array[list_idx, self.ss, 1]
                com_ddotx_1 = com_acc_array[list_idx, self.ss, 0]
                com_ddoty_1 = com_acc_array[list_idx, self.ss, 1]
                l_cmd_x_1, l_cmd_y_1 = self._to_com_frame(l_x, l_y, com_x_1, com_y_1, com_theta)
                l_cmd_dotx_1, l_cmd_doty_1 = self._to_com_frame(0, 0, com_dotx_1, com_doty_1, com_theta)
                l_cmd_accx_1, l_cmd_accy_1 = self._to_com_frame(0, 0, com_ddotx_1, com_ddoty_1, com_theta)
                l_cmd_z_1 = 0 if predict_idx == 0 else l_cmd_z
                l_cmd_theta_1 = l_theta / 2
                r_cmd_x_1, r_cmd_y_1 = self._to_com_frame(r_x, r_y, com_x_1, com_y_1, com_theta)
                r_cmd_dotx_1, r_cmd_doty_1 = self._to_com_frame(0, 0, com_dotx_1, com_doty_1, com_theta)
                r_cmd_accx_1, r_cmd_accy_1 = self._to_com_frame(0, 0, com_ddotx_1, com_ddoty_1, com_theta)
                r_cmd_z_1 = 0 if predict_idx == 0 else r_cmd_z
                r_cmd_theta_1 = r_theta / 2

                com_x_2 = com_traj_array[list_idx, self.ds_begin, 0]
                com_y_2 = com_traj_array[list_idx, self.ds_begin, 1]
                com_dotx_2 = com_vel_array[list_idx, self.ds_begin, 0]
                com_doty_2 = com_vel_array[list_idx, self.ds_begin, 1]
                com_ddotx_2 = com_acc_array[list_idx, self.ds_begin, 0]
                com_ddoty_2 = com_acc_array[list_idx, self.ds_begin, 1]
                l_cmd_x_2, l_cmd_y_2 = self._to_com_frame(l_x, l_y, com_x_2, com_y_2, com_theta)
                l_cmd_dotx_2, l_cmd_doty_2 = self._to_com_frame(0, 0, com_dotx_2, com_doty_2, com_theta)
                l_cmd_accx_2, l_cmd_accy_2 = self._to_com_frame(0, 0, com_ddotx_2, com_ddoty_2, com_theta)
                l_cmd_z_2 = 0
                l_cmd_theta_2 = l_theta
                r_cmd_x_2, r_cmd_y_2 = self._to_com_frame(r_x, r_y, com_x_2, com_y_2, com_theta)
                r_cmd_dotx_2, r_cmd_doty_2 = self._to_com_frame(0, 0, com_dotx_2, com_doty_2, com_theta)
                r_cmd_accx_2, r_cmd_accy_2 = self._to_com_frame(0, 0, com_ddotx_2, com_ddoty_2, com_theta)
                r_cmd_z_2 = 0
                r_cmd_theta_2 = r_theta

                com_x_3 = com_traj_array[list_idx, self.ds_end, 0]
                com_y_3 = com_traj_array[list_idx, self.ds_end, 1]
                com_dotx_3 = com_vel_array[list_idx, self.ds_end, 0]
                com_doty_3 = com_vel_array[list_idx, self.ds_end, 1]
                com_ddotx_3 = com_acc_array[list_idx, self.ds_end, 0]
                com_ddoty_3 = com_acc_array[list_idx, self.ds_end, 1]
                l_cmd_x_3, l_cmd_y_3 = self._to_com_frame(l_x, l_y, com_x_3, com_y_3, com_theta)
                l_cmd_dotx_3, l_cmd_doty_3 = self._to_com_frame(0, 0, com_dotx_3, com_doty_3, com_theta)
                l_cmd_accx_3, l_cmd_accy_3 = self._to_com_frame(0, 0, com_ddotx_3, com_ddoty_3, com_theta)
                l_cmd_z_3 = 0
                l_cmd_theta_3 = l_theta
                r_cmd_x_3, r_cmd_y_3 = self._to_com_frame(r_x, r_y, com_x_3, com_y_3, com_theta)
                r_cmd_dotx_3, r_cmd_doty_3 = self._to_com_frame(0, 0, com_dotx_3, com_doty_3, com_theta)
                r_cmd_accx_3, r_cmd_accy_3 = self._to_com_frame(0, 0, com_ddotx_3, com_ddoty_3, com_theta)
                r_cmd_z_3 = 0
                r_cmd_theta_3 = r_theta

                command = StepCommand(
                    idx = predict_idx,
                    l_cmd_init = l_cmd_init,
                    r_cmd_init = r_cmd_init,
                    l_cmd_1 = [l_cmd_x_1, l_cmd_y_1, l_cmd_z_1, l_cmd_theta_1, l_cmd_dotx_1, l_cmd_doty_1, l_cmd_accx_1, l_cmd_accy_1],
                    l_cmd_2 = [l_cmd_x_2, l_cmd_y_2, l_cmd_z_2, l_cmd_theta_2, l_cmd_dotx_2, l_cmd_doty_2, l_cmd_accx_2, l_cmd_accy_2],
                    l_cmd_3 = [l_cmd_x_3, l_cmd_y_3, l_cmd_z_3, l_cmd_theta_3, l_cmd_dotx_3, l_cmd_doty_3, l_cmd_accx_3, l_cmd_accy_3],
                    r_cmd_1 = [r_cmd_x_1, r_cmd_y_1, r_cmd_z_1, r_cmd_theta_1, r_cmd_dotx_1, r_cmd_doty_1, r_cmd_accx_1, r_cmd_accy_1],
                    r_cmd_2 = [r_cmd_x_2, r_cmd_y_2, r_cmd_z_2, r_cmd_theta_2, r_cmd_dotx_2, r_cmd_doty_2, r_cmd_accx_2, r_cmd_accy_2],
                    r_cmd_3 = [r_cmd_x_3, r_cmd_y_3, r_cmd_z_3, r_cmd_theta_3, r_cmd_dotx_3, r_cmd_doty_3, r_cmd_accx_3, r_cmd_accy_3]
                )
                self.__command_list.append(command)

    @property
    def command_list(self) -> list[StepCommand]:
        return self.__command_list

def main():
    footstep = StepGenerator()
    planner = DCMPlanner()
    commander = StepCommander()

    goal_step = 5
    footstep.init()
    footstep.update(1, 1, -1)
    footstep.update(2, 1, -1)
    footstep.update(3, 1, -1)
    footstep.update(4, 1, -1)
    footstep.update(goal_step, 1, -1)
    steps = footstep.list()

    print("Initial steps".center(60, "="))
    for step in steps:
        print(f'Step {step.nStep}: Foot {"left" if step.is_left() else "right"}, Position {step.pos}')
    
    planner.compute(steps)
    com_traj = planner.com_traj_array
    com_vel = planner.com_vel_array
    com_acc = planner.com_acc_array

    commander.command(steps, com_traj, com_vel, com_acc)

    print("Position commands".center(60, "="))

    for cmd in commander.command_list:
        print(cmd)
    
    print("Plt current steps".center(60, "="))
    print("Current support foot:", steps[goal_step].nStep, "left" if steps[goal_step].is_left() else "right")        
    planner.visualize(threeD=False, step_list=steps)
    
    goal_step = 10
    footstep.update(7, 1, 1)
    footstep.update(8, 1, 1)
    footstep.update(9, 1, 1)
    footstep.update(goal_step, 1, 1)
    steps = footstep.list()
    
    planner.compute(steps)
    com_traj = planner.com_traj_array
    com_vel = planner.com_vel_array
    com_acc = planner.com_acc_array

    commander.command(steps, com_traj, com_vel, com_acc)

    print("Position commands".center(60, "="))

    for cmd in commander.command_list:
        print(cmd)

    print("Plt current steps".center(60, "="))
    print("Current support foot:", steps[goal_step].nStep, "left" if steps[goal_step].is_left() else "right")    
    planner.visualize(threeD=False, step_list=steps)

    print("nStep error test".center(60, "*"))
    try:
        footstep.update(goal_step, 1, -1)    
    except Exception as e:
        print(f'[{type(e).__name__}] {e}')
    try:        
        footstep.update(goal_step + 3, 1, 1)
    except Exception as e:
        print(f'[{type(e).__name__}] {e}')
    try:
        footstep.reset()
        footstep.update(1, 1, 1)
    except Exception as e:
        print(f'[{type(e).__name__}] {e}')


if __name__ == '__main__':
    main()