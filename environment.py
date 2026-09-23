from datetime import datetime

import numpy as np
import math
import pymsis
from utils import conversions as conv
import constants as const

def j2_acceleration_m_s2(r_eci_m: np.ndarray) -> np.ndarray:
    """
    Calculate the J2 perturbative acceleration.

    Arguments:
    r_eci_m (np.ndarray): x, y, z -> Earth-Centered Inertial position vector [m].

    Returns:
    [ax, ay, az] (np.ndarray): acceleration due to Earth's J2 oblateness perturbation [m/s^2].
    """
    r_eci_m = np.asarray(r_eci_m, dtype=float)
    r_norm_m = np.linalg.norm(r_eci_m)

    if r_eci_m.shape != (3,) or r_norm_m == 0.0:
        raise ValueError("r_eci_m must be a nonzero 3D vector.")

    x, y, z = r_eci_m
    
    mu_m3_s2 = const.EARTH_MU_m3_s2
    r_eq_m = const.EARTH_RADIUS_EQ_m
    j2 = 1.08262668e-3

    z2_over_r2 = (z**2) / (r_norm_m**2)
    factor = - 3 / 2 * (mu_m3_s2 * j2 * r_eq_m**2) / (r_norm_m**5)

    ax = factor * x * (1 - 5 * z2_over_r2)
    ay = factor * y * (1 - 5 * z2_over_r2)
    az = factor * z * (3 - 5 * z2_over_r2)

    return np.array([ax, ay, az])

def NRLMSIS_atmospheric_density_kg_m3(
    r_eci_m: np.ndarray,
    epoch: datetime,
    f107: float = 150.0,
    f107a: float = 150.0,
    ap: float = 4.0,
) -> float:
    """
    Calculate atmospheric density using NRLMSIS.

    Arguments:
    r_eci_m (np.ndarray): x, y, z -> Earth-Centered Inertial position vector [m].
    epoch (datetime): YYYY, MM, DD, hh, mm, ss -> UTC datetime of the spacecraft position.
    f107 (float): Daily 10.7 cm solar radio flux index [sfu].
    f107a (float): 81-day centered average of the 10.7 cm solar radio flux index [sfu].
    ap (float): Planetary geomagnetic activity index [2 nT].

    Returns:
    float: Atmospheric mass density [kg/m^3].
    """
    r_eci_m = np.asarray(r_eci_m, dtype=float)
    r_norm_m = np.linalg.norm(r_eci_m)

    if r_eci_m.shape != (3,) or r_norm_m <= const.EARTH_RADIUS_POLAR_m:
        raise ValueError("r_eci_m must be a 3D position vector with magnitude greater than the Earth's radius.")
    
    if type(epoch) != datetime:
        raise ValueError("epoch must be of type datetime.")

    lla_coords = conv.eci_to_lla(r_eci_m, epoch)

    dates = np.array([epoch], dtype="datetime64[us]")
    lons = np.array([lla_coords[0]])
    lats = np.array([lla_coords[1]])
    alts = np.array([lla_coords[2] / 1000.0])

    output = pymsis.calculate(
        dates,
        lons,
        lats,
        alts,
        f107s=np.array([f107]),
        f107as=np.array([f107a]),
        aps=np.array([ap]),
        version=2.1,
    )

    total_mass_density_kg_m3 = output[0][0]

    return float(total_mass_density_kg_m3)

def approximate_atmospheric_density_kg_m3(r_eci_m: np.ndarray) -> float:
    """
    Calculate the approximate atmopheric density at position r_eci_m.
    """
    r_eci_m = np.asarray(r_eci_m, dtype=float)
    semi_major = const.EARTH_RADIUS_EQ_m
    semi_minor = const.EARTH_RADIUS_POLAR_m

    dist = np.sqrt(r_eci_m[0] ** 2 + r_eci_m[1] ** 2)
    geocentric_latitude = math.atan2(r_eci_m[2], dist)

    ellipsoid_height = semi_major * semi_minor / (math.sqrt(semi_minor**2 + (semi_major**2 - semi_minor**2) * (math.sin(geocentric_latitude))**2))
    altitude = (np.linalg.norm(r_eci_m) - ellipsoid_height) / 1000

    # Air density model most valid from 180-500km altitudes (see https://www.spaceacademy.net.au/watch/debris/atmosmod.htm):
    F10 = 115 # Average value of solar EUV flux proxy, solar radio 10cm flux
    Ap = 10 # Average value of geomagnetic activity proxy, geomagnetic Ap index
    T = 900 + 2.5 * (F10 - 70) + 1.5 * Ap
    mu = 27 - 0.012 * (altitude - 200)
    H = T / mu

    density = 6 * (10 ** (-10)) * math.exp(-1 * (altitude - 175)/H)
    
    return density

def calc_atm_velocity_m_s(r_eci_m: np.ndarray, earth_ang_velocity_rad_s: np.ndarray):
    """
    Calculate the Earth's average atmospheric velocity at a given location in the ECI frame.

    Arguments:
    r_eci_m (np.ndarray): x, y, z -> ECI position vector [m].
    earth_ang_velocity_rad_s (ndarray): -> ECI angular velocity vector of Earth [rad/s].

    Returns:
    atm_velocity_m_s (np.ndarray): ECI average atmospheric velocity [m/s].
    """
    r_eci_m = np.asarray(r_eci_m, dtype=float)
    earth_ang_velocity_rad_s = np.asarray(earth_ang_velocity_rad_s, dtype=float)

    if r_eci_m.shape != (3,) or np.linalg.norm(r_eci_m) <= const.EARTH_RADIUS_POLAR_m:
        raise ValueError("r_eci_m must be a 3D position vector with magnitude greater than the Earth's radius.")
    
    if earth_ang_velocity_rad_s.shape != (3,):
        raise ValueError("earth_ang_velocity_rad_s must be a 3D vector.")

    atm_velocity_m_s = np.cross(earth_ang_velocity_rad_s, r_eci_m)

    return atm_velocity_m_s

def aerodynamic_drag_perturbation_m_s2(
    velocity_m_s: np.ndarray,
    velocity_atm_m_s: np.ndarray,
    air_kg_m3: float,
    drag_coeff: float,
    area_m_2: float,
    mass_kg: float,
) -> np.ndarray:
    """
    Calculate acceleration due to aerodynamic drag.

    Arguments:
    velocity_m_s (np.ndarray): vx, vy, vz -> Satellite velocity vector [m/s].
    velocity_atm_m_s (np.ndarray): v_atm_x, v_atm_y, v_atm_z -> Atmosphere velocity vector [m/s].
    air_kg_m3 (float): Air density [kg/m^3].
    drag_coeff (float): Drag coefficient.
    area_m_2 (float): Cross-sectional area [m^2].
    mass_kg (float): Satellite mass in kg.
    
    Returns:
    [ax, ay, az] (np.ndarray): Acceleration vector due to drag [m/s^2].
    """
    velocity_m_s = np.asarray(velocity_m_s, dtype=float)
    velocity_atm_m_s = np.asarray(velocity_atm_m_s, dtype=float)

    if velocity_m_s.shape != (3,) or velocity_atm_m_s.shape != (3,):
        raise ValueError("velocity_m_s and velocity_atm_m_s must be 3D vectors.")

    if air_kg_m3 < 0:
        raise ValueError("Air density must be non-negative.")
    elif drag_coeff < 0:
        raise ValueError("Drag coefficient must be non-negative.")
    elif area_m_2 < 0:
        raise ValueError("Cross-sectional area must be non-negative.")
    elif mass_kg <= 0:
        raise ValueError("Mass must be positive.")

    v_relative_m_s = velocity_m_s - velocity_atm_m_s
    speed = np.linalg.norm(v_relative_m_s)

    factor = -(0.5 * air_kg_m3 * drag_coeff * area_m_2) / mass_kg

    return factor * speed * v_relative_m_s
