from numpy import pi

class Gravity():
    def __init__(self) -> None:
        pass

    def __call__(self, mass: float):
        return 


class EarthGravity(Gravity):
    def __init__(self):
        self.G = 6.672*10^(-11)
        self.M = 5.972*10^(24)
        self.RE = 6378000
        self.J2 = 1.083*10^(-3)
        self.J22 = 1.86*10^(-6)
        self.L22 = -15*pi/180

    def __call__(self, mass, pos: tuple) -> float:
        
        
        
        return super().__call__(mass)