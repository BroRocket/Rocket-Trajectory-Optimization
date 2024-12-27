
import numpy as np

from RocketTrajectoryOptimization.Ressources.Datasheets.constants import RADIUS_EARTH
import RocketTrajectoryOptimization.Ressources.Position.tools as tools

class Atmosphere():
    def __init__(self):

        self.g0 = 9.80665  # Standard gravity (m/s²)
        self.R = 287.05    # Specific gas constant for dry air (J/(kg·K))

        self.layers = [
        {"h_b": 0, "T_b": 288.15, "P_b": 101325, "L": -0.0065},
        {"h_b": 11000, "T_b": 216.65, "P_b": 22632.1, "L": 0.0},
        {"h_b": 20000, "T_b": 216.65, "P_b": 5474.89, "L": 0.001},
        {"h_b": 32000, "T_b": 228.65, "P_b": 868.019, "L": 0.0028},
        {"h_b": 47000, "T_b": 270.65, "P_b": 110.906, "L": 0.0},
        {"h_b": 51000, "T_b": 270.65, "P_b": 66.9389, "L": -0.0028},
        {"h_b": 71000, "T_b": 214.65, "P_b": 3.95642, "L": -0.002}
        ]

    def __call__(self, pos: np.ndarray) -> float:
        positions = tools.cartesian_to_spheircal(pos)
        altitude = positions[0] - RADIUS_EARTH
        if altitude > 175000:
            return 0 

        for layer in self.layers:
            if altitude >= layer["h_b"]:
                h_b = layer["h_b"]
                T_b = layer["T_b"]
                P_b = layer["P_b"]
                L = layer["L"]

        if L != 0:  # Non-isothermal layer
            T = T_b + L * (altitude - h_b)
            P = P_b * (T / T_b) ** (-self.g0 / (L * self.R))
        else:  # Isothermal layer
            T = T_b
            P = P_b * np.exp(-self.g0 * (altitude - h_b) / (self.R * T))
        
        rho = P / (self.R * T)
        return rho

       

if __name__ == "__main__":
    atm = Atmosphere()
    for i in range(1, 500):
        print(f"{i*1000} => {atm(i*1000)}")


