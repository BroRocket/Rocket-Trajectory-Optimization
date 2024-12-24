import math
import numpy as np

import RocketTrajectoryOptimization.Ressources.Position.tools as tools

# class Gravity():
#     def __init__(self) -> None:
#         pass

#     def __call__(self, mass: float):
#         return 

class EarthGravity():
    def __init__(self):
        self.G = 6.672*10^(-11)
        self.M = 5.972*10^(24)
        self.RE = 6378000
        self.J2 = 1.083*10^(-3)
        self.J22 = 1.86*10^(-6)
        self.L22 = -15*math.pi/180
        self.MU = self.G*self.M

    def __call__(self, pos: np.ndarray) -> float:
        '''Expects cartesian coordinates'''
        position = tools.orbitalspherical_conv(tools.cartesian_to_spheircal(pos))
        '''lattitude is polar angle  whihc is pos[2]'''
        fg_r = -(self.MU/position[0]**2) - (3/2)*self.MU*((self.RE/position[0]**2)**2)*(self.J2*(3*(math.cos(position[2])**2)-1)) + 9*self.MU*((self.RE/position[0]**2)**2)*(self.J22*math.cos(2(position[1]-self.L22))*math.cos(position[2])**2)
        fg_t = (3*self.MU*(self.RE**2)*self.J2*math.cos(position[2])*math.sin(position[2]))/(position[0]**4) - (6*self.MU*(self.RE**2)*self.J22*math.cos(2(position[1]-self.L22))*math.cos(position[2])*math.sin(position[2]))/(position[0]**4)
        fg_p = (6*self.MU*(self.RE**2)*math.sin(2(position[1]-self.L22))*(math.cos(position[2])**2))/(position[0]*math.sin((90*math.pi/180)-position[2]))
        
        accels = np.array([fg_r, fg_t, fg_p])
        accels = tools.spherical_to_cartesian(accels)

        return accels