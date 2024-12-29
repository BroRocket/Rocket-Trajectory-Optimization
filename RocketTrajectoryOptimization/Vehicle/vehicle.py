
import numpy as np

from RocketTrajectoryOptimization.Ressources.Position.state import State
from RocketTrajectoryOptimization.Vehicle.stage import Stage


class Vehicle():
    def __init__(self, payload_mass: float, stages: list[Stage]):
        '''Note tages should appear in the list in order they are used for rocket. ie first stgae is 0th element'''
        self.stages = stages
        self.payload_mass = payload_mass
        self.mass = payload_mass
        self.structural_mass = payload_mass
        self.height = 0
        self.diameter = 0
        self.Cd = 0
        for stage in stages:
            self.mass += stage.mass
            self.structural_mass += stage.structural_mass
            self.height += stage.height
            if stage.diameter > self.diameter:
                self.diameter = stage.diameter
            if self.Cd < stage.Cd:
                self.Cd = stage.Cd
        self.STAGES_CPY = self.stages.copy()

    def reset(self):
        self.stages = self.STAGES_CPY
        self.structural_mass = self.payload_mass
        self.height = 0
        self.diameter = 0
        self.Cd = 0
        for stage in self.stages:
            self.mass += stage.mass
            self.structural_mass += stage.structural_mass
            self.height += stage.height
            if stage.diameter > self.diameter:
                self.diameter = stage.diameter
            if self.Cd < stage.Cd:
                self.Cd = stage.Cd

    def update(self, thrust_dir: np.ndarray, dt: float):
        if self.stages[0].update_mass(dt) is False:
            mass = self.stage_sep(dt)
            if len(self.stages) == 0:
                return np.array([0, 0, 0], dtype=np.float64)
        else:
            self.mass = sum([stage.mass for stage in self.stages])
            mass =  self.mass + 0.5*dt*self.stages[0].engine.MDOT # half mass whihc is average that accel acts on over dt
        return self.stages[0].engine(mass, thrust_dir)

    def stage_sep(self, dt: float):
        self.structural_mass -= self.stages[0].structural_mass
        self.height -= self.stages[0].height
        old_stage = self.stages.pop(0)
        if len(self.stages) == 0:
            return False
        self.mass = self.payload_mass
        self.diameter = 0
        self.Cd = 0
        for stage in self.stages:
            self.mass += stage.mass
            if stage.diameter > self.diameter:
                self.diameter = stage.diameter
            if self.Cd < stage.Cd:
                self.Cd = stage.Cd
        return self.mass + 0.5*dt*old_stage.engine.MDOT
