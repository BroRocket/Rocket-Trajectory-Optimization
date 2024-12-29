
import numpy as np
from RocketTrajectoryOptimization.Ressources.Position import tools

class State():
    def __init__(self, radius: float, lattitude: float, longitude: float) -> None:
        orbital_cords = np.array([radius, lattitude, longitude], np.float64)
        spherical_cords = tools.orbitalspherical_conv(orbital_cords)
        self.pos = tools.spherical_to_cartesian(spherical_cords) # verify this is wokring 

        self.vel = np.array([0, 0, 0], np.float64)
        self.accel = np.array([0, 0, 0], np.float64)

        self.POSITIONS = [self.pos]
        self.VELOCITIES = [self.vel]
        self.ACCELERATIONS = [self.accel]

    def update(self, accel: np.ndarray, dt: float):
        '''use rk4 scheme, could be imporved but would reqire computing accel over and over whihc is tricky witht eh ay vehicle is set up'''

        self.accel = accel
        self.pos += self.vel * dt + 0.5*self.accel*dt**2
        self.vel += self.accel * dt
        
        self.ACCELERATIONS.append(self.accel)
        self.VELOCITIES.append(self.vel)
        self.POSITIONS.append(self.pos)
