
import math
import numpy as np

import RocketTrajectoryOptimization.Ressources.Position.tools as tools

class Atmosphere():
    def __init__(self):
        pass

    def Temp(self, height: float) -> float:
        if height < 11000:
            return 15.04 - 0.00649*height
        elif height < 25000:
            return -56.46
        else:
            return -131.21 + 0.00299*height

    def Pressure(self, height: float) -> float:
        if height < 11000:
            return 101.29*((self.Temp(height) + 273.15)/288.08)**5.256
        elif height < 25000:
            return 22.65*math.exp(1.73 - 0.000157*height)
        else:
            return 2.488*((self.Temp(height) + 273.15)/216.6)**(-11.388)
    
    def Density(self, height: float) -> float:
        rho = self.Pressure(height) / (0.2869 * (self.Temp(height) - 273.15))
        if rho < 0:
            return 0
        return rho

    def __call__(self, pos: np.ndarray) -> float:
        positions = tools.cartesian_to_spheircal(pos)
        height = positions[0]
        return self.Density(height)

