
import numpy as np

class State():
    def __init__(self, pos: np.ndarray = np.array([0, 0, 0])) -> None:
        '''All in cartesian'''
        self.pos = pos
        self.vel = np.array([0, 0, 0])
        self.accel = np.array([0, 0, 0])

    def update(self):
        pass