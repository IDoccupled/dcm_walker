from step import Step
import numpy as np
import matplotlib.pyplot as plt

class SingleStepGenerator:
    '''
    Create a single-step based on previous step and cmd input.
    
    :param step_length: Step length in meters.
    :type step_length: float
    :param step_width: Step width in meters.
    :type step_width: float
    :param step_theta: Step yaw in degrees (positive = left turn).
    :type step_theta: float
    '''
    def __init__(self, 
                 step_length: float = 0.05, 
                 step_width: float = 0.058,
                 step_theta: float = 5.0):
        self.length = step_length
        self.width = step_width
        self.theta = step_theta
        self.current_theta = 0.0

    def gen(self, pre_step: Step, cmd_vel: float, cmd_rot: float) -> Step:
        '''
        Generate next step based on previous step and cmd input.
        
        :param pre_step: Previous step.
        :type pre_step: Step
        :param cmd_vel: Linear velocity command, 0.0-1.0.
        :type cmd_vel: float
        :param cmd_rot: Rotational velocity command from -1 to 1. Plus for left turn.
        :type cmd_rot: float
        :return: Next step.
        :rtype: Step
        '''
        footSide = 'right' if pre_step.is_left() else 'left'
        nStep = pre_step.nStep + 1

        delta_x = self.length * cmd_vel
        delta_y = self.width * (-1 if footSide == 'right' else 1)
        
        '''
        prevent crossing steps during turning
        '''
        if footSide == 'right' and cmd_rot > 0:
            delta_theta = 0
        elif footSide == 'left' and cmd_rot < 0:
            delta_theta = 0
        else:
            delta_theta = np.deg2rad(cmd_rot*self.theta)
        self.current_theta += delta_theta

        rot = np.array([
            [np.cos(self.current_theta), -np.sin(self.current_theta)],
            [np.sin(self.current_theta),  np.cos(self.current_theta)],
        ])

        delta_p = rot @ np.array([[delta_x], [delta_y]])
        pre_pos = pre_step.pos
        pos = np.array([pre_pos[0], pre_pos[1], 0]) + np.array([delta_p[0,0], delta_p[1,0], delta_theta])
        # print(f'delta x = {delta_x}, delta y = {delta_y}')
        # print(f"{delta_p[0,0]:.3f}, {delta_p[1,0]:.3f}, {np.rad2deg(delta_theta):.2f}, ({pos[0]:.3f}, {pos[1]:.3f}, {np.rad2deg(pos[2]):.2f})")

        return Step(footSide, pos, nStep)
    
def main():
    ssg = SingleStepGenerator()
    steps = []
    first_step = Step('l', np.array([0.0, 0.0, 0.0]), 0)
    steps.append(first_step)
    second_step = ssg.gen(first_step, 1.0, -0.5)
    steps.append(second_step)
    third_step = ssg.gen(second_step, 1.0, -0.5)
    steps.append(third_step)
    forth_step = ssg.gen(third_step, 1.0, 0.0)
    steps.append(forth_step)
    fifth_step = ssg.gen(third_step, 1.0, 0.0)
    steps.append(fifth_step)
    new_step = ssg.gen(fifth_step, 0.5, 0.0)
    steps.append(new_step)
    new1_step = ssg.gen(new_step, 0.0, 0.0)
    steps.append(new1_step)
    new2_step = ssg.gen(new1_step, 0.0, 0.0)
    steps.append(new2_step)
    print("First Step:", first_step.pos)
    print("Second Step:", second_step.pos)
    print("Third Step:", third_step.pos)
    for step in steps:
        plt.plot(step.pos[0], step.pos[1], 'ro' if step.is_left() else 'bo')
        plt.text(step.pos[0], step.pos[1], f'{"L" if step.is_left() else "R"}, {step.nStep}', fontsize=9, ha='center', va='bottom')
    plt.title("Step Positions")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.grid(True)
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    main()