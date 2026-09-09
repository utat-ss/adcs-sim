"""
constants.py contains all the commonly used constants throughout the software
"""

G_m3_kgs2 = 6.6743 * 10**-11 # m^3 / (kg * s^2)  --- Gravitational Constant
M_kg = 5.97219 * 10**24 # kg   ---   Earth's Mass
EARTH_MU_m3_s2 = G_m3_kgs2*M_kg # m^3 / s^2   ---   G*M,the gravitational parameter

# Aliases for backward compatibility
G = G_m3_kgs2
M = M_kg
mu = EARTH_MU_m3_s2

# Earth parameters
EARTH_RADIUS_m = 6378137.0 # m --- Earth equatorial radius
EARTH_RADIUS_km = 6378.137 # km
EARTH_J2 = 1.08262668e-3 # dimensionless --- Earth J2 oblateness harmonic coefficient
EARTH_OMEGA_rad_s = 7.2921159e-5 # rad/s --- Earth rotation rate