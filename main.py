import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from RocketTrajectoryOptimization.Ressources.Phys.gravity import EarthGravity
from RocketTrajectoryOptimization.Ressources.Phys.drag import Drag

if __name__ == "__main__":
    TIME_STEP = 1  # seconds
    TOTAL_TIME = 2.5*3600 # 24 hours in seconds
    NUM_STEPS = int(TOTAL_TIME / TIME_STEP)

    # Initialize the gravity class
    gravity = EarthGravity()
    drag = Drag(0.7)

    # Initial position and velocity for a circular orbit
    # Approximate altitude: 500 km above Earth's surface
    initial_position = np.array([gravity.RE + 200000, 0, 0], dtype=np.float64)  # 500 km above Earth's surface
    orbital_speed = np.sqrt(gravity.MU / np.linalg.norm(initial_position))  # Circular orbit speed
    print(orbital_speed)
    initial_velocity = np.array([0, 7755, 0], dtype=np.float64)  # Perpendicular to position vector

    positions = np.zeros((NUM_STEPS, 3))
    velocities = np.zeros((NUM_STEPS, 3))
    accelerations = np.zeros((NUM_STEPS, 3))

    # Set initial conditions
    positions[0] = initial_position
    velocities[0] = initial_velocity
    accelerations[0] = gravity(initial_position)

    # Time integration using Velocity Verlet
    for step in range(1, NUM_STEPS):
        # Get acceleration from gravity
        grav_acceleration = gravity(positions[step - 1])
        #print(grav_acceleration)
        drag_acceleration = drag(positions[step - 1], velocities[step - 1], 1.2, 150)
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
    x = gravity.RE * np.outer(np.cos(u), np.sin(v))
    y = gravity.RE * np.outer(np.sin(u), np.sin(v))
    z = gravity.RE * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='blue', alpha=0.6)

    # Plot the orbit
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], color='red', label='Orbit')

    # Plot acceleration vectors occasionally
    step_interval = 500  # Adjust this to change vector frequency
    for step in range(0, NUM_STEPS, step_interval):
        ax.quiver(
            positions[step, 0], positions[step, 1], positions[step, 2],
            accelerations[step, 0], accelerations[step, 1], accelerations[step, 2],
            color='green', length=gravity.RE / 20, normalize=True, label='Acceleration' if step == 0 else ""
        )

    # Set plot limits
    axis_limit = gravity.RE + 1100000  # Slightly larger than orbit altitude
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