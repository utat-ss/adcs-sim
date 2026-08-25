from datetime import datetime

import numpy as np
import pymsis
from utils import conversions as conv
import constants as const

def j2_acceleration_m_s2(r_eci_m: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    Calculate the J2 perturbative acceleration.

    Arguments:
    r_eci_m (tuple): x, y, z -> Earth-Centered Inertial position vector [meters].

    Returns:
    ax, ay, az (tuple): acceleration due to Earth's J2 oblateness perturbation [m/s^2].
    """
    x, y, z = r_eci_m
    r_norm_m = np.linalg.norm(r_eci_m)
    mu_m3_s2 = const.EARTH_MU_m3_s2
    r_eq_m = const.EARTH_RADIUS_EQ_m
    j2 = 1.08262668e-3

    if r_norm_m < r_eq_m + 100000:
        raise ValueError("Position vector must be at least 100km above Earth's surface.")

    z2_over_r2 = (z**2) / (r_norm_m**2)
    factor = - 3 / 2 * (mu_m3_s2 * j2 * r_eq_m**2) / (r_norm_m**5)

    ax = factor * x * (1 - 5 * z2_over_r2)
    ay = factor * y * (1 - 5 * z2_over_r2)
    az = factor * z * (3 - 5 * z2_over_r2)

    return (ax, ay, az)

def NRLMSIS_atmospheric_density_kg_m3(
    r_eci_m: tuple[float, float, float],
    epoch: datetime,
    f107: float = 150.0,
    f107a: float = 150.0,
    ap: float = 4.0,
) -> float:
    """
    Calculate atmospheric density using NRLMSIS.

    Arguments:
    r_eci_m (tuple): x, y, z -> Earth-Centered Inertial position vector [meters].
    epoch (datetime): YYYY, MM, DD, hh, mm, ss -> UTC datetime of the spacecraft position.
    f107 (float): Daily 10.7 cm solar radio flux index [sfu].
    f107a (float): 81-day centered average of the 10.7 cm solar radio flux index [sfu].
    ap (float): Planetary geomagnetic activity index [2 nT].

    Returns:
    float: Atmospheric mass density [kg/m^3].
    """
    r_eci_m = np.asarray(r_eci_m, dtype=float)
    r_norm_m = np.linalg.norm(r_eci_m)

    if len(r_eci_m) != 3:
        raise ValueError("r_eci_m must be a tuple of length 3.")
    
    if type(epoch) != datetime:
        raise ValueError("epoch must be of type datetime.")

    if r_norm_m < const.EARTH_RADIUS_POLAR_m:
        raise ValueError("Position vector norm must not be less than the Earth's radius.")

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

def calc_atm_velocity_m_s():
    """
    """
    pass

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

    velocity_m_s (np.ndarray): vx, vy, vz -> Satellite velocity vector [m/s].
    velocity_atm_m_s (np.ndarray): v_atm_x, v_atm_y, v_atm_z -> Atmosphere velocity vector [m/s].
    air_kg_m3 (float): Air density [kg/m^3].
    drag_coeff (float): Drag coefficient.
    area_m_2 (float): Cross-sectional area [m^2].
    mass_kg (float): Satellite mass in kg.
    
    Returns:
    ax, ay, az (np.ndarray): Acceleration vector due to drag [m/s^2].
    """
    if mass_kg <= 0:
        raise ValueError("Mass must be positive.")
    elif area_m_2 < 0 or drag_coeff < 0:
        raise ValueError("Cross-sectional area must be non-negative.")
    elif air_kg_m3 < 0:
        raise ValueError("Air density must be non-negative.")
    elif drag_coeff < 0:
        raise ValueError("Drag coefficient must be non-negative.")

    v_relative_m_s = velocity_m_s - velocity_atm_m_s
    speed = np.linalg.norm(v_relative_m_s)

    factor = -(0.5 * air_kg_m3 * drag_coeff * area_m_2) / mass_kg

    return factor * speed * v_relative_m_s
