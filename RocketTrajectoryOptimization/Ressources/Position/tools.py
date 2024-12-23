import math

# check raidnas versus degrees


def orbital_to_spherical(pos: tuple) -> tuple:
    '''(r, lattitutude, Longitude), note lat and long in degrees'''
    if len(pos) != 3:
        raise Exception(f"Position tuple is not the correct length. Has length {len(pos)}")
    PSI = (90 - pos[1])*math.pi/180
    return (pos[0], pos[1], PSI)

def cartesian_to_spheircal(pos: tuple) -> tuple:
    '''(x, y, z)'''
    if len(pos) != 3:
        raise Exception(f"Position tuple is not the correct length. Has length {len(pos)}")
    R = (pos[0]**2 + pos[1]**2 + pos[2]**2)**0.5
    THETA = math.atan(pos[1]/pos[0])
    PSI = math.acos(pos[2]/R)
    return (R, THETA, PSI)

def spherical_to_cartesian(pos: tuple) -> tuple:
    '''(r, theta, psi)'''
    if len(pos) != 3:
        raise Exception(f"Position tuple is not the correct length. Has length {len(pos)}")
    X = pos[0]*math.sin(pos[2])*math.cos(pos[1])
    Y = pos[0]*math.sin(pos[2])*math.sin(pos[1])
    Z = pos[0]*math.cos(pos[2])
    return (X, Y, Z)