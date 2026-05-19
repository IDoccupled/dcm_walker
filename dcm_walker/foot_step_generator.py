import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_THIS_DIR)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import numpy as np
from dcm_walker.single_step_generator import SingleStepGenerator
from dcm_walker.step import Step
import matplotlib.pyplot as plt

class nStepError(Exception):
    pass

class StepGenerator:
    '''
    Create and update steps based on cmd input.
    
    :param step_length: Max step length in [meters].
    :type step_length: float
    :param step_width: Foot separation width in [meters].
    :type step_width: float
    :param step_theta: Max step yaw in [degrees].
    :type step_theta: float
    :param predict_horizon: Number of steps to be predicted.
    :type predict_horizon: int
    '''
    def __init__(self, 
                 step_length: float = 0.05, 
                 step_width: float = 0.058,
                 step_theta: float = 10.0,
                 predict_horizon: int = 3):
        self.length = step_length
        self.width = step_width
        self.theta = step_theta
        self.horizon = predict_horizon + 1  # always including final standing step
        self.__list = []
        self.__nStep = 0 # current step idx
        self.__inited = False

        self.ssg = SingleStepGenerator(step_length, step_width, step_theta)

    def init(self):
        '''
        Initialize first double support step at origin.
        '''
        # if len(self.__list) == 0:
        #     first_step = Step('l', np.array([0.0, self.width/2, 0.0]), self.__nStep)
        # else:
        #     first_step = self.ssg.gen(self.__list[-1], 0, 0)


        first_step = Step('l', np.array([0.0, self.width/2, 0.0]), self.__nStep)

        self.__nStep += 1
        second_step = self.ssg.gen(first_step, 0, 0)
        self.__nStep += 1
        self.__list = [first_step, second_step]
        self.__inited = True
        
    def update(self, cur_nStep:int, cmd_vel:float, cmd_rot:float) -> list[Step]:
        '''
        Give current step idx to make sure prediction 
        is always from current real step.
        
        :param cmd_vel: Linear velocity command, 0.0-1.0.
        :type cmd_vel: float
        :param cmd_rot: Rotational velocity command from -1 to 1. Plus for left turn.
        :type cmd_rot: float
        '''
        if not self.__inited:
            raise nStepError(f'StepGenerator not initialized. Call init() first.')
        elif cur_nStep < self.__nStep - 1 and cur_nStep != 1:
            raise nStepError(f'Error nStep index {cur_nStep}: Given nStep {cur_nStep} must be larger than internal nStep {self.__nStep - 2} and smaller than predict horizon {self.__nStep + 1}.')
        elif cur_nStep > self.__nStep:
            raise nStepError(f'Error nStep index {cur_nStep}: Robot already stopped.')
        
        if len(self.__list) < self.horizon:
            temp_list = self.__list.copy()
        else:
            temp_list = self.__list[: - self.__nStep + cur_nStep - self.horizon + 2].copy()
        for _ in range(self.horizon-2): # excluding penult deceleration step + final standing step
            '''
            we always generate new steps from the current real step so
            we don't update __nStep in for loop
            '''
            pre_step = temp_list[-1]
            if _ == 0:                    
                next_step = self.ssg.gen(pre_step, cmd_vel, cmd_rot)
                temp_list.append(next_step)
            else:
                next_step = self.ssg.gen(pre_step, cmd_vel, 0)
                temp_list.append(next_step)
        pre_step = temp_list[-1]
        penult_step = self.ssg.gen(pre_step, cmd_vel/2, 0)
        temp_list.append(penult_step)
        pre_step = temp_list[-1]
        final_step = self.ssg.gen(pre_step, 0, 0)
        temp_list.append(final_step)
        if len(self.__list) < self.horizon:
            self.__list = temp_list
        else:
            self.__list = self.__list[: - self.__nStep + cur_nStep - self.horizon + 2] + temp_list[-self.horizon:]
        self.__nStep = cur_nStep + 2
            
    def reset(self):
        '''
        Reset step list and nStep counter.
        '''
        self.__list = []
        self.__nStep = 0
        self.__inited = False
        self.ssg.current_theta = 0.0
        
    def list(self) -> list[Step]:
        return self.__list
    
    @property
    def inited(self) -> bool:
        return self.__inited
    
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
    steps = footstep.list()
    theta = 0.0
    for step in steps:
        print(f'Step {step.nStep}: Foot {"left" if step.is_left() else "right"}, Position {step.pos}')
        plt.plot(step.pos[0], step.pos[1], 'ro' if step.is_left() else 'bo')
        plt.text(step.pos[0], step.pos[1], f'{"L" if step.is_left() else "R"}, {step.nStep}', fontsize=9, ha='center', va='bottom')
        arrow_length = 0.03
        theta += step.pos[2]
        plt.arrow(step.pos[0], step.pos[1], arrow_length * np.cos(theta), arrow_length * np.sin(theta), head_width=0.005, head_length=0.003, fc='gray', ec='gray')
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    ax.axis('equal')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()