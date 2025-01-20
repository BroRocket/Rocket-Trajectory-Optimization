
import numpy as np
from scipy.optimize import root

from RocketTrajectoryOptimization.Vehicle.vehicle import Vehicle
from RocketTrajectoryOptimization.Ressources.Phys import gravity, drag
from RocketTrajectoryOptimization.Ressources.Position import tools, state


class GravityTurnSim():
    def __init__(self, vehicle: Vehicle, orbit_altitude: float, orbit_inclination: float, launch_longitude: float, launch_lattitude: float):
        '''note currently assumes circular orbit at that altitude'''
        self.GRAVITY = gravity.EarthGravity()
        self.DRAG = drag.Drag()

        self.ROCKET = vehicle
        self.orbit_altitude = orbit_altitude
        self.orbit_inclination = orbit_inclination
        self.desired_velocity = np.sqrt(self.GRAVITY.MU/(self.GRAVITY.RE+self.orbit_altitude))
        self.launch_longitude = launch_longitude
        self.launch_lattitude = launch_lattitude

        if self.launch_lattitude < self.orbit_inclination:
            raise Exception("Cannot insert into orbit with lower inclination then lattitude")

        #find unit vector in direction of orbit
        #assume that RAAND is 0 at first 
        if self.orbit_inclination != 0:
            right_ascension = np.radians(self.launch_longitude) + np.arcsin(np.tan(self.launch_lattitude)/np.tan(self.orbit_inclination))
        else:
            right_ascension = np.radians(self.launch_longitude) + np.arcsin(np.tan(self.launch_lattitude))
        # fucking up this here in conversion
        self.launch_pos = np.array([self.GRAVITY.RE, self.launch_lattitude, self.launch_longitude], dtype=np.float64) #orbital
        self.launch_pos = tools.spherical_to_cartesian(tools.orbitalspherical_conv(self.launch_pos)) #cartesian now
        self.RAAN_pos = tools.spherical_to_cartesian(np.array([self.GRAVITY.RE, np.pi/2, right_ascension], dtype=np.float64))
        orbital_vector = np.cross(self.launch_pos, self.RAAN_pos)
        if np.linalg.norm(orbital_vector) == 0: # handles equitorial launch and orbit, on't be correct for incline equitorial orbits need to fix this
            self.orbital_unit_vector = np.array([0, 0, 1], dtype=np.float64)
        else:
            self.orbital_unit_vector = tools.unit_vector(orbital_vector)
        

    def iniatialize(self):
        '''should initalize relative psoiton of rocket to ECIF'''
        
        self.STATE = state.State(self.GRAVITY.RE, self.launch_lattitude, self.launch_longitude)
        self.ROCKET.reset()
        #self.ROCKET.stage_delay = stage_delay


    def run_launch(self, manuever: list):

        print(manuever)
        pitchover_height = manuever[0]
        pitchover_angle = manuever[1]
        burn_time = manuever[2]
        t = 0
        maneuver_completed = False
        initial_mass = self.ROCKET.mass
        self.iniatialize()

        while True:

            velocity = np.cross(self.orbital_unit_vector, self.STATE.pos)
            velocity_unit_vector= tools.unit_vector(velocity)
            if np.linalg.norm(self.STATE.pos) - self.GRAVITY.RE == self.orbit_altitude and self.desired_velocity * velocity_unit_vector == self.STATE.vel:
                break
            if len(self.ROCKET.stages) == 0: # check if no delta v left
                break
            
            #run time step, update position
            if maneuver_completed == False and (tools.cartesian_to_spheircal(self.STATE.pos))[0] - self.GRAVITY.RE >= pitchover_height:
                # adjust direction of thrust
                rotation_axis = np.cross(self.STATE.vel, velocity_unit_vector)
                rotation_axis = tools.unit_vector(rotation_axis)
                skew_symmetric = np.array([[0, -rotation_axis[2], rotation_axis[1]], [rotation_axis[2], 0, -rotation_axis[0]], [-rotation_axis[1], rotation_axis[0], 0]])
                rotation_matrix = np.identity(3) + np.sin(pitchover_angle)*skew_symmetric + (1 - np.cos(pitchover_angle))*(skew_symmetric**2)
                thrust_dir = np.dot(rotation_matrix, velocity_unit_vector)
                thrust_dir = tools.unit_vector(thrust_dir)
                thrust_accel = self.ROCKET.update(thrust_dir, self.dt) # check if this is working
                maneuver_completed = True
            else:
                if np.linalg.norm(self.STATE.vel) == 0:
                    thrust_dir = tools.unit_vector(self.STATE.pos)
                else:
                    thrust_dir = tools.unit_vector(self.STATE.vel)
                thrust_accel = self.ROCKET.update(thrust_dir, self.dt)
            
            gravity_accel = self.GRAVITY(self.STATE.pos)
            drag_accel = self.DRAG(self.STATE.pos, self.STATE.vel, self.ROCKET.Cd, self.ROCKET.diameter, self.ROCKET.mass)
            accel = thrust_accel + gravity_accel + drag_accel
            self.STATE.update(accel, self.dt)

            t += self.dt
            if t > burn_time:
                print("had fuel left")
                break
            if t > 10000:
                raise Exception("Program has run a sim for over 10000 seconds")

        velocity = np.cross(self.orbital_unit_vector, self.STATE.pos)
        velocity = self.desired_velocity * tools.unit_vector(velocity)
        residual_velocity = self.STATE.vel - velocity
        altitude = (tools.cartesian_to_spheircal(self.STATE.pos))[0] - self.GRAVITY.RE
        res1 = np.sum(residual_velocity)/3
        res2 = self.orbit_altitude - altitude
        res3 = 0
        residuals = [res1, res2, res3]
        print(residuals)
        return residuals
       
        

    def optimize(self, pitchover_height_guess, pitchover_angle_guess, burn_time, dt: float):
        '''
        start with intial guess,
        run sim see residuals
        
        '''
        # use root finding
        self.dt = dt
        sol = root(self.run_launch, [pitchover_height_guess, pitchover_angle_guess, burn_time]).x
        print(sol)

