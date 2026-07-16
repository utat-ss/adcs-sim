import numpy as np

def j2_acceleration_m_s2(r_eci_m: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    Calculate the J2 perturbative acceleration.

    Arguments:
    r_eci_m (tuple): x, y, z -> Earth-Centered Inertial position vector [meters].

    Returns:
    tuple: ax, ay, az -> acceleration due to Earth's J2 oblateness perturbation [m/s^2].
    """
    x, y, z = r_eci_m
    r_norm_m = np.linalg.norm(r_eci_m)
    mu_m3_s2 = 3.986004418e14
    r_eq_m = 6378136.3
    j2 = 1.08262668e-3

    if r_norm_m < r_eq_m + 100000:
        raise ValueError("Position vector must be at least 100km above Earth's surface.")

    z2_over_r2 = (z**2) / (r_norm_m**2)
    factor = - 3 / 2 * (mu_m3_s2 * j2 * r_eq_m**2) / (r_norm_m**5)

    ax = factor * x * (1 - 5 * z2_over_r2)
    ay = factor * y * (1 - 5 * z2_over_r2)
    az = factor * z * (3 - 5 * z2_over_r2)

    return (ax, ay, az)


def atmospheric_density_kg_m3(
    r_eci_m: tuple[float, float, float],
) -> float:
    """
    Calculate atmospheric density using NASA's Earth Atmosphere Model in metric units.

    Arguments:
    r_eci_m (tuple): x, y, z Earth-Centered Inertial position vector [m].

    Returns:
    float: Atmospheric density [kg/m^3].
    """
    r_norm_m = np.linalg.norm(r_eci_m)
    r_eq_m = 6378136.3

    if r_norm_m == 0:
        raise ValueError("Position vector must be nonzero.")

    altitude_m = r_norm_m - r_eq_m

    if altitude_m < 0:
        raise ValueError("Altitude must be nonnegative.")

    # Atmospheric effects neglected above the stratosphere
    if altitude_m > 50000:
        return 0.0

    # Troposphere
    elif altitude_m <= 11000:
        temperature_c = 15.04 - 0.00649 * altitude_m
        pressure_kpa = 101.29 * ((temperature_c + 273.1) / 288.08) ** 5.256

    # Lower stratosphere
    elif altitude_m <= 25000:
        temperature_c = -56.46
        pressure_kpa = 22.65 * np.exp(1.73 - 0.000157 * altitude_m)

    # Upper stratosphere
    else:
        temperature_c = -131.21 + 0.00299 * altitude_m
        pressure_kpa = 2.488 * ((temperature_c + 273.1) / 216.6) ** -11.388

    density_kg_m3 = pressure_kpa / (0.2869 * (temperature_c + 273.1))

    return float(density_kg_m3)