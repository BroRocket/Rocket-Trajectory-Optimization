
from RocketTrajectoryOptimization.Vehicle.engine import Engine


class Stage():
    def __init__(self, diameter: float, height: float, Cd: float, dry_mass: float, propellant_mass: float, engine: Engine):
        self.mass = dry_mass + propellant_mass
        self.structural_mass = dry_mass
        self.diameter = diameter
        self.Cd = Cd
        self.height = height
        self.engine = engine
    
    # def update(self, dt: float):
    #     if self.update_mass(dt) is False:
    #         return False
    #     else:
    #         return True

    def update_mass(self, dt: float):
        self.mass -= dt * self.engine.MDOT
        if self.mass < self.structural_mass:
            self.mass = self.structural_mass
            return False
        else:
            return True