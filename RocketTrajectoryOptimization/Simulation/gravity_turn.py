
import numpy as np

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
        right_ascension = np.radians(self.launch_longitude) + np.arcsin(np.tan(self.launch_lattitude)/np.tan(self.orbit_inclination))
        self.launch_pos = np.array([self.GRAVITY.RE, self.launch_longitude, self.launch_lattitude]) #orbital
        self.launch_pos = tools.spherical_to_cartesian(tools.orbitalspherical_conv(self.launch_pos)) #cartesian now
        self.RAAN_pos = tools.spherical_to_cartesian(np.array([self.GRAVITY.RE, right_ascension, np.pi/2]))
        orbital_vector = np.cross(self.launch_pos, self.RAAN_pos)
        self.orbital_unit_vector = tools.unit_vector(orbital_vector)
        

    def iniatialize(self):
        '''should initalize relative psoiton of rocket to ECIF'''
        
        self.STATE = state.State(self.GRAVITY.RE, self.launch_lattitude, self.launch_longitude)
        self.ROCKET.reset()


    def run_launch(self, pitchover_height, pitchover_angle, dt: float):

        t = 0
        maneuver_completed = False

        while True:

            velocity = np.cross(self.orbital_unit_vector, self.STATE.pos)
            velocity_unit_vector= tools.unit_vector(velocity)
            if np.linalg.norm(self.STATE.pos) - self.GRAVITY.RE == self.orbit_altitude and self.desired_velocity * velocity_unit_vector == self.STATE.vel:
                break
            if len(self.ROCKET.stages) == 0:
                break
            
            #run time step, update position
            if maneuver_completed == False and (tools.cartesian_to_spheircal(self.STATE.pos))[0] - self.GRAVITY.RE >= pitchover_height:
                # adjust direction of thrust 
                thrust_accel = self.ROCKET.update(tools.unit_vector(self.STATE.vel), dt) # need to figure this out still
            else:
                thrust_accel = self.ROCKET.update(tools.unit_vector(self.STATE.vel), dt)
            
            gravity_accel = self.GRAVITY(self.STATE.pos)
            drag_accel = self.DRAG(self.STATE.pos, self.STATE.vel, self.ROCKET.Cd, self.ROCKET.diameter, self.ROCKET.mass)
            accel = thrust_accel + gravity_accel + drag_accel
            self.STATE.update(accel, dt)

            t += dt

            if t > 10000:
                raise Exception("Program has run a sim for over 10000 seconds")

        velocity = np.cross(self.orbital_unit_vector, self.STATE.pos)
        velocity = self.desired_velocity * tools.unit_vector(velocity)
        residual_velocity = self.STATE.vel - velocity
        altitude = (tools.cartesian_to_spheircal(self.STATE.pos))[0] - self.GRAVITY.RE
        res1 = np.sum(residual_velocity)/3
        res2 = self.orbit_altitude - altitude
        

            # check if no delta v left


    def optimize(self, pitchover_height_guess, pitchover_angle_guess, dt: float):
        '''
        start with intial guess,
        run sim see residuals
        
        '''


