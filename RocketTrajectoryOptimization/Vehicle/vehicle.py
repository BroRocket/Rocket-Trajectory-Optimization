from RocketTrajectoryOptimization.Ressources.Position.state import State
from RocketTrajectoryOptimization.Vehicle.stage import Stage


class Vehicle():
    def __init__(self, payload_mass: float, stages: list[Stage]):
        '''Note tages should appear in the list in order they are used for rocket. ie first stgae is 0th element'''
        self.stages = stages
        self.mass = payload_mass
        self.structural_mass = payload_mass
        self.height = 0
        self.diameter = 0
        for stage in stages:
            self.mass += stage.mass
            self.structural_mass += stage.structural_mass
            self.height += stage.height
            if stage.diameter > self.diameter:
                stage.diameter = self.diameter
        
    def update(self, dt: float):
        if self.stages[0].update_mass(dt) is False:
            return self.stage_sep()
        self.mass = sum([stage.mass for stage in self.stages])
        return self.mass + 0.5*dt*self.stages[0].engine.MDOT # half mass whihc is average that accel acts on over dt

    def stage_sep(self, dt: float):
        self.structural_mass -= self.stages[0].structural_mass
        self.height -= self.stages[0].height
        old_stage = self.stages.pop(0)
        self.mass = sum([stage.mass for stage in self.stages])
        self.diameter = max([stage.diameter for stage in self.stages])
        return self.mass + 0.5*dt*old_stage.engine.MDOT
