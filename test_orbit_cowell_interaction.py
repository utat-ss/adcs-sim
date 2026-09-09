import numpy as np
from scipy.integrate import solve_ivp
from constants import EARTH_MU_m3_s2, EARTH_RADIUS_m, EARTH_J2, EARTH_OMEGA_rad_s
from orbit import cowell_motion


def j2_accel(r_vec: np.ndarray) -> np.ndarray:

    # Distance from Earth's center
    r = np.linalg.norm(r_vec)
    
    z2_r2 = (r_vec[2] / r) ** 2

    factor = 1.5 * EARTH_J2 * EARTH_MU_m3_s2 * (EARTH_RADIUS_m ** 2) / (r ** 5)

    # Accerlation in each direction
    ax = factor * r_vec[0] * (5.0 * z2_r2 - 1.0)
    ay = factor * r_vec[1] * (5.0 * z2_r2 - 1.0)
    az = factor * r_vec[2] * (5.0 * z2_r2 - 3.0) #Different formula

    return np.array([ax, ay, az])


def drag_accel(r_vec: np.ndarray, v_vec: np.ndarray) -> np.ndarray:

    # Satellite properties (change if needed)
    Cd = 2.2         # Drag coefficient
    area_m2 = 0.04   # Area of satellite [m^2]
    mass_kg = 4.0    # Mass [kg]

    rho = 3.0e-12    # Density [kg/m^3] at 400km

    # Velocity relative to the rotating atmosphere
    earth_spin = np.array([0.0, 0.0, EARTH_OMEGA_rad_s])
    v_rel = v_vec - np.cross(earth_spin, r_vec)
    v_rel_mag = np.linalg.norm(v_rel)

    # Drag formula
    F_drag = -0.5 * Cd * rho * v_rel_mag * v_rel * area_m2
    a_drag = F_drag / mass_kg

    return a_drag


def cowell_rhs(t: float, x: np.ndarray) -> np.ndarray:
    r_vec = x[0:3]  # Position vector
    v_vec = x[3:6]  # Velocity vector

    p_total = j2_accel(r_vec) + drag_accel(r_vec, v_vec)

    return cowell_motion(x, p_total)


def test_cowell_leo_j2_drag_duration_ephemeris():

    altitude_m = 400_000.0 # Altitude (change if needed)
    a0 = EARTH_RADIUS_m + altitude_m  

    v0 = np.sqrt(EARTH_MU_m3_s2 / a0)   # Orbit speed
    inc_rad = np.deg2rad(51.6)          # Orbit inclination of the ISS

    r0_vec = np.array([a0, 0.0, 0.0])   # Position vector
    v0_vec = np.array([0.0, v0 * np.cos(inc_rad), v0 * np.sin(inc_rad)]) # Velocity vector
    x0 = np.hstack([r0_vec, v0_vec])    # Initial state vector


    orbit_period_s = 2 * np.pi * np.sqrt(a0 ** 3 / EARTH_MU_m3_s2) # Orbital period in seconds

    n_orbits_short = round((3 * 3600.0) / orbit_period_s)    # ~3 hours
    n_orbits_medium = round((24 * 3600.0) / orbit_period_s)  # ~24 hours

    t_short = n_orbits_short * orbit_period_s
    t_medium = n_orbits_medium * orbit_period_s

   
    sol = solve_ivp(cowell_rhs, t_span=(0.0, t_medium), y0=x0, method="DOP853", t_eval=[0.0, t_short, t_medium], rtol=1e-10, atol=1e-12)
    assert sol.status == 0, f"Failed: {sol.message}"

    x_t0 = sol.y[:, 0] # Initial state
    x_short = sol.y[:, 1] # State at ~3 hours
    x_medium = sol.y[:, 2] # State at ~24 hours

    # Test to make sure altitude is within range
    alt_short_km = (np.linalg.norm(x_short[0:3]) - EARTH_RADIUS_m) / 1000.0
    alt_medium_km = (np.linalg.norm(x_medium[0:3]) - EARTH_RADIUS_m) / 1000.0
    assert 380.0 <= alt_short_km <= 420.0, f"Short window altitude out of range: {alt_short_km} km"
    assert 380.0 <= alt_medium_km <= 420.0, f"Medium window altitude out of range: {alt_medium_km} km"


