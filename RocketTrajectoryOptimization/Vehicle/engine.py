
import numpy as np

class Engine():
    def __init__(self, mass_flow_rate: float, ISP: float): # maybe add thrut curve for srb later
        self.MDOT = mass_flow_rate
        self.ISP = ISP
        self.g0 = 9.80665

    def __call__(self, vehicle_mass: float, thrust_dir: np.ndarray) -> np.ndarray:

        thrust_mag = self.MDOT * self.g0 * self.ISP
        thrust = thrust_dir*thrust_mag
        thurst_accel = thrust / vehicle_mass

        return thurst_accel
