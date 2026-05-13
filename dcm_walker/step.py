import numpy as np
class Step:
    '''
    A 2D foot step representation.
    
    :param footSide: 'l' or 'r' for left or right foot.
    :type footSide: str
    :param pos: x, y(meters), theta (radians) position of the foot step.
    :type pos: np.ndarray[3]
    :param nStep: A unique step idx in the sequence.
    :type nStep: int
    '''
    def __init__(self, footSide: str, pos: np.ndarray[3], nStep: int):
        self.__side = 'left' if footSide.lower() in ['left', 'l'] else 'right'
        self.__pos = pos
        self.__nStep = nStep
    def __str__(self):
        return f"Step {self.__nStep}: {self.__side}, {self.__pos}"
    def is_left(self) -> bool:
        return self.__side == 'left'
    @property
    def pos(self) -> np.ndarray[3]:
        return self.__pos    
    @property
    def nStep(self) -> int:
        return self.__nStep
    @property
    def footSide(self) -> str:
        return self.__side