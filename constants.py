"""
constants.py contains all the commonly used constants throughout the software
"""

G_m3_kgs2 = 6.6743 * 10**-11 # m^3 / (kg * s^2)  --- Gravitational Constant
M_kg = 5.97219 * 10**24 # kg   ---   Earth's Mass
EARTH_MU_m3_s2 = G_m3_kgs2*M_kg # m^3 / s^2   ---   G*M,the gravitational parameter

SUN_RADIUS_m = 695700000.0 # m --- average radius of the sun

EARTH_RADIUS_EQ_m = 6378137.0 # m --- equatorial radius of the Earth (semi-major axis of WGS 84 ellipsoid)
EARTH_RADIUS_POLAR_m = 6356752.314245 # m --- radius of the Earth at poles (semi-minor axis of WGS 84 ellipsoid)

MOON_RADIUS_m = 1737400.0 # m --- average radius of the Moon
