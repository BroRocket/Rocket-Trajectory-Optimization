import math
import numpy as np

# check raidnas versus degrees

def unit_vector(vec: np.ndarray):
    return vec/np.linalg.norm(vec)

def orbitalspherical_conv(pos: np.ndarray) -> np.ndarray:
    '''(r, Longitude, lattitude), note lat and long in degrees'''
    if len(pos) != 3:
        raise Exception(f"Position tuple is not the correct length. Has length {len(pos)}")
    PSI = (90 - pos[2]*180/math.pi)*math.pi/180 # just relace with pi/2
    return np.array([pos[0], pos[1], PSI])

def cartesian_to_spheircal(pos: np.ndarray) -> np.ndarray:
    '''(x, y, z)'''
    if len(pos) != 3:
        raise Exception(f"Position tuple is not the correct length. Has length {len(pos)}")
    
    R = np.sqrt((pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]))
    THETA = np.arctan(pos[1]/pos[0]) 
    PSI = np.arccos(pos[2]/R)
    return np.array([R, THETA, PSI])

def spherical_to_cartesian(pos: np.ndarray) -> np.ndarray:
    '''(r, theta, psi)'''
    if len(pos) != 3:
        raise Exception(f"Position tuple is not the correct length. Has length {len(pos)}")
    X = pos[0]*math.sin(pos[1])*math.cos(pos[2])
    Y = pos[0]*math.sin(pos[1])*math.sin(pos[2])
    Z = pos[0]*math.cos(pos[1])
    return np.array([X, Y, Z])

def accel_spherical_to_cartesian(accel_spherical: np.ndarray, position: np.ndarray) -> np.ndarray:
    """
    Convert acceleration from spherical to Cartesian coordinates, considering the spacecraft's position.
    
    :param accel_spherical: Spherical acceleration components [a_r, a_theta, a_phi]
    :param position: Position vector of the spacecraft in Cartesian coordinates [x, y, z]
    :return: Cartesian acceleration vector [a_x, a_y, a_z]
    """
    # Unpack spherical components
    a_r, a_theta, a_phi = accel_spherical

    # Unpack position components
    x, y, z = position
    R = np.linalg.norm(position)

    # Compute spherical basis vectors
    # Radial unit vector
    e_r = unit_vector(position)

    # Azimuthal unit vector (handle divide-by-zero case for x^2 + y^2 == 0)
    xy_norm = np.sqrt(x**2 + y**2)
    if xy_norm > 0:
        e_theta = np.array([-y / xy_norm, x / xy_norm, 0])
    else:
        e_theta = np.array([0, 0, 0])  # Azimuthal unit vector undefined at poles

    # Polar unit vector
    e_phi = np.array([
        -x * z / (R * xy_norm) if xy_norm > 0 else 0,
        -y * z / (R * xy_norm) if xy_norm > 0 else 0,
        xy_norm / R
    ])

    # Transform acceleration
    a_cartesian = a_r * e_r + a_theta * e_theta + a_phi * e_phi
    return a_cartesian