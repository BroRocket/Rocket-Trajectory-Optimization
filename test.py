
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from RocketTrajectoryOptimization.Simulation.gravity_turn import GravityTurnSim
from RocketTrajectoryOptimization.Vehicle import stage, engine, vehicle

eng = engine.Engine(2740.26212, 297.5)
stg1 = stage.Stage(3.7, 47.7, 0.8, 25600, 395700, eng)
eng2 = engine.Engine(287.4544842, 348)
stg2 = stage.Stage(3.7, 13.8, 0.8, 3900, 92670, eng2)
rocket = vehicle.Vehicle(10000, [stg1, stg2])
# siome resson R is at z posiiton either conversion issue or iniatialization issue
sim = GravityTurnSim(rocket, 300000, 0, 0, 0)
sim.dt = 0.1

#sim.optimize(200, np.radians(87), 435, 0.05)

#maybe ad solution blockers
sim.run_launch([165, np.radians(80), 430]) # not sure if gimbaling is done right


def plot_trajectory(positions, velocities, accelerations, vector_interval=50, vector_scale=0.1):
    """
    Plot the 3D trajectory of the rocket with velocity and acceleration vectors.

    :param positions: List of position vectors (numpy arrays) over time.
    :param velocities: List of velocity vectors (numpy arrays) over time.
    :param accelerations: List of acceleration vectors (numpy arrays) over time.
    :param vector_interval: Interval at which to plot velocity and acceleration vectors.
    :param vector_scale: Scaling factor for velocity and acceleration vectors.
    """
    # Convert lists to numpy arrays for easy manipulation
    positions = np.array(positions)
    velocities = np.array(velocities)
    accelerations = np.array(accelerations)

    # Create a 3D plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the trajectory
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label="Rocket Trajectory", color='blue')

    # Add velocity and acceleration vectors
    for i in range(0, len(positions), vector_interval):
        # Velocity vectors
        ax.quiver(
            positions[i, 0], positions[i, 1], positions[i, 2],  # Starting point
            velocities[i, 0], velocities[i, 1], velocities[i, 2],  # Direction
            color='green', length=vector_scale, normalize=True, label="Velocity Vector" if i == 0 else ""
        )
        # Acceleration vectors
        ax.quiver(
            positions[i, 0], positions[i, 1], positions[i, 2],  # Starting point
            accelerations[i, 0], accelerations[i, 1], accelerations[i, 2],  # Direction
            color='red', length=vector_scale, normalize=True, label="Acceleration Vector" if i == 0 else ""
        )

    # Add labels and legend
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Rocket Trajectory with Velocity and Acceleration Vectors")
    ax.legend()

    # Adjust the plot limits to fit the trajectory
    all_coords = np.vstack([positions, velocities * vector_scale, accelerations * vector_scale])
    max_limit = np.max(np.abs(all_coords)) * 1.1
    ax.set_xlim([-max_limit, max_limit])
    ax.set_ylim([-max_limit, max_limit])
    ax.set_zlim([-max_limit, max_limit])

    plt.show()

print(sim.STATE.vel)
print(np.linalg.norm(sim.STATE.vel))
print(sim.STATE.pos[0] - sim.GRAVITY.RE)
plot_trajectory(sim.STATE.POSITIONS, sim.STATE.VELOCITIES, sim.STATE.ACCELERATIONS, 20, 0.3)

TIME_STEP = 1  # seconds
TOTAL_TIME = 7*3600 # 24 hours in seconds
NUM_STEPS = int(TOTAL_TIME / TIME_STEP)

positions = np.zeros((NUM_STEPS, 3))
velocities = np.zeros((NUM_STEPS, 3))
accelerations = np.zeros((NUM_STEPS, 3))

# Set initial conditions
positions[0] = sim.STATE.pos
velocities[0] = sim.STATE.vel
accelerations[0] = sim.STATE.accel

# Time integration using Velocity Verlet
for step in range(1, NUM_STEPS):
    # Get acceleration from gravity
    grav_acceleration = sim.GRAVITY(positions[step - 1])
    #print(grav_acceleration)
    drag_acceleration = sim.DRAG(positions[step - 1], velocities[step - 1], sim.ROCKET.Cd, sim.ROCKET.diameter, sim.ROCKET.mass)
    #print(drag_acceleration)
    acceleration = grav_acceleration + drag_acceleration
    #print(acceleration)
    
    # Update velocity and position using Euler method
    velocities[step] = velocities[step - 1] + acceleration * TIME_STEP
    positions[step] = positions[step - 1] + velocities[step] * TIME_STEP


# Plot the orbit and acceleration vectors
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# Plot the Earth
u, v = np.linspace(0, 2 * np.pi, 100), np.linspace(0, np.pi, 100)
x = sim.GRAVITY.RE * np.outer(np.cos(u), np.sin(v))
y = sim.GRAVITY.RE * np.outer(np.sin(u), np.sin(v))
z = sim.GRAVITY.RE * np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_surface(x, y, z, color='blue', alpha=0.6)

# Plot the orbit
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], color='red', label='Orbit')

# Plot acceleration vectors occasionally
step_interval = 500  # Adjust this to change vector frequency
for step in range(0, NUM_STEPS, step_interval):
    ax.quiver(
        positions[step, 0], positions[step, 1], positions[step, 2],
        accelerations[step, 0], accelerations[step, 1], accelerations[step, 2],
        color='green', length=sim.GRAVITY.RE / 20, normalize=True, label='Acceleration' if step == 0 else ""
    )

# Set plot limits
axis_limit = sim.GRAVITY.RE + 1100000  # Slightly larger than orbit altitude
ax.set_xlim([-axis_limit, axis_limit])
ax.set_ylim([-axis_limit, axis_limit])
ax.set_zlim([-axis_limit, axis_limit])

# Add labels and legend
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Satellite Orbit and Acceleration Vectors")
ax.legend()

plt.show()

