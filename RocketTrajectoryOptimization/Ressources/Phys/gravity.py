import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


#import RocketTrajectoryOptimization.Ressources.Position.tools as tools
def orbitalspherical_conv(pos: np.ndarray) -> np.ndarray:
    '''(r, lattitutude, Longitude), note lat and long in degrees'''
    if len(pos) != 3:
        raise Exception(f"Position tuple is not the correct length. Has length {len(pos)}")
    PSI = (90 - pos[2]*180/math.pi)*math.pi/180
    return np.array([pos[0], pos[1], PSI])

def cartesian_to_spheircal(pos: np.ndarray) -> np.ndarray:
    '''(x, y, z)'''
    if len(pos) != 3:
        raise Exception(f"Position tuple is not the correct length. Has length {len(pos)}")
    
    i = pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]
    print(math.sqrt(i))
    R = math.sqrt((pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]))
    THETA = math.atan(pos[1]/pos[0])
    PSI = math.acos(pos[2]/R)
    return np.array([R, THETA, PSI])

def spherical_to_cartesian(pos: np.ndarray) -> np.ndarray:
    '''(r, theta, psi)'''
    if len(pos) != 3:
        raise Exception(f"Position tuple is not the correct length. Has length {len(pos)}")
    X = pos[0]*math.sin(pos[2])*math.cos(pos[1])
    Y = pos[0]*math.sin(pos[2])*math.sin(pos[1])
    Z = pos[0]*math.cos(pos[2])
    return np.array([X, Y, Z])

class EarthGravity():
    def __init__(self):
        self.G = 6.672*10**(-11)
        self.M = 5.972*10**(24)
        self.RE = 6378000
        self.J2 = 1.083*10**(-3)
        self.J22 = 1.86*10**(-6)
        self.L22 = -15*math.pi/180
        self.MU = self.G*self.M

    def __call__(self, pos: np.ndarray) -> float:
        '''Expects cartesian coordinates'''
        print(pos)
        position = cartesian_to_spheircal(pos)
        position = orbitalspherical_conv(position)
        '''lattitude is polar angle  whihc is pos[2]'''
        fg_r = -(self.MU/position[0]**2) - (3/2)*self.MU*((self.RE/position[0]**2)**2)*(self.J2*(3*((math.cos(position[2])**2)-1))) + 9*self.MU*((self.RE/position[0]**2)**2)*(self.J22*math.cos(2*(position[1]-self.L22))*math.cos(position[2])**2)
        fg_t = (3*self.MU*(self.RE**2)*self.J2*math.cos(position[2])*math.sin(position[2]))/(position[0]**4) - (6*self.MU*(self.RE**2)*self.J22*math.cos(2*(position[1]-self.L22))*math.cos(position[2])*math.sin(position[2]))/(position[0]**4)
        fg_p = (6*self.MU*(self.RE**2)*math.sin(2*(position[1]-self.L22))*(math.cos(position[2])**2))/(position[0]*math.sin((90*math.pi/180)-position[2]))
        
        accels = np.array([fg_r, fg_t, fg_p])
        accels = spherical_to_cartesian(accels)

        return accels
    

if __name__ == "__main__":
    TIME_STEP = 0.1  # seconds
    TOTAL_TIME = 5  # 24 hours in seconds
    NUM_STEPS = int(TOTAL_TIME / TIME_STEP)

    # Initialize the gravity class
    gravity = EarthGravity()

    # Initial position and velocity for a circular orbit
    # Approximate altitude: 500 km above Earth's surface
    initial_position = np.array([gravity.RE + 500000, 0, 0], dtype=np.float64)  # 500 km above Earth's surface
    orbital_speed = np.sqrt(gravity.MU / np.linalg.norm(initial_position))  # Circular orbit speed
    initial_velocity = np.array([0, orbital_speed, 0], dtype=np.float64)  # Perpendicular to position vector

    positions = np.zeros((NUM_STEPS, 3))
    velocities = np.zeros((NUM_STEPS, 3))
    accelerations = np.zeros((NUM_STEPS, 3))

    # Set initial conditions
    positions[0] = initial_position
    velocities[0] = initial_velocity
    accelerations[0] = gravity(initial_position)

    # Time integration using Velocity Verlet
    for step in range(1, NUM_STEPS):
        # Compute acceleration
        acceleration = gravity(positions[step - 1])

        # Update position
        positions[step] = positions[step - 1] + velocities[step - 1] * TIME_STEP + 0.5 * acceleration * TIME_STEP**2

        # Compute new acceleration and update velocity
        new_acceleration = gravity(positions[step])
        velocities[step] = velocities[step - 1] + 0.5 * (acceleration + new_acceleration) * TIME_STEP
        accelerations[step] = new_acceleration

    # Plot the orbit and acceleration vectors
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # # Plot the Earth
    # u, v = np.linspace(0, 2 * np.pi, 100), np.linspace(0, np.pi, 100)
    # x = gravity.RE * np.outer(np.cos(u), np.sin(v))
    # y = gravity.RE * np.outer(np.sin(u), np.sin(v))
    # z = gravity.RE * np.outer(np.ones(np.size(u)), np.cos(v))
    # ax.plot_surface(x, y, z, color='blue', alpha=0.6)

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
    axis_limit = gravity.RE + 700000  # Slightly larger than orbit altitude
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