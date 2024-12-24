
import numpy as np
import math

from RocketTrajectoryOptimization.Ressources.Datasheets.atmosphere import Atmosphere
from RocketTrajectoryOptimization.Ressources.Position.tools import *

class Drag():
    def __init__(self, Cd: float) -> None:
        self.Cd = Cd
        self.atmosphere = Atmosphere()

    def __call__(self, pos: tuple, vel: tuple, vehicle_diamter: float, vehicle_mass: float):
        density = self.atmosphere(pos)

        # this may change
        proj_area = math.pi*(vehicle_diamter/2)**2

        drag_force = (-0.5*self.Cd*density*proj_area*np.linalg.norm(vel)**2) * unit_vector(vel)
        drag_accel = drag_force/vehicle_mass

        return drag_accel