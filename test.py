
import numpy as np

from RocketTrajectoryOptimization.Simulation.gravity_turn import GravityTurnSim
from RocketTrajectoryOptimization.Vehicle import stage, engine, vehicle

eng = engine.Engine(500, 400)
stg1 = stage.Stage(2.5, 15, 0.8, 1000, 10000, eng)
rocket = vehicle.Vehicle(10, [stg1])
# siome resson R is at z posiiton either conversion issue or iniatialization issue
sim = GravityTurnSim(rocket, 150000, 0, 0, 0)
sim.dt = 0.1

#maybe ad solution blockers
sim.run_launch([10000, np.radians(20)])