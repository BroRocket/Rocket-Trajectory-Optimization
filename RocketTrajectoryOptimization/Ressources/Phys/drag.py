
import numpy as np
import math

from RocketTrajectoryOptimization.Ressources.Datasheets.atmosphere import Atmosphere
from RocketTrajectoryOptimization.Ressources.Position.tools import *

class Drag():
    def __init__(self) -> None:
        self.atmosphere = Atmosphere()

    def __call__(self, pos: tuple, vel: tuple, Cd: float, vehicle_diamter: float, vehicle_mass: float):
        density = self.atmosphere(pos)

        # this may change
        proj_area = math.pi*(vehicle_diamter/2)**2
        vel_mag = np.linalg.norm(vel)
        if vel_mag == 0:
            return np.array([0, 0 ,0], dtype=np.float64)
        else:
            drag_force = (-0.5*Cd*density*proj_area*vel_mag**2) * unit_vector(vel)
            drag_accel = drag_force/vehicle_mass

        return drag_accel