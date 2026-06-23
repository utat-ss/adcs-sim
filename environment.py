import numpy as np

def j2_acceleration_m_s2(
    r_eci_m: tuple[float, float, float],
    mu_m3_s2: float = 3.986004418e14,
    r_eq_m: float = 6378136.3,
    j2: float = 1.08262668e-3,
) -> tuple[float, float, float]:
    """Calculate the J2 perturbative acceleration.

    Arguments:
    r_eci_m (tuple): x, y, z -> Earth-Centered Inertial position vector [meters].
    mu_m3_s2 (float): Standard gravitational parameter [m^3/s^2].
    r_eq_m (float): Mean equatorial radius of the body [meters].
    j2 (float): Second zonal harmonic coefficient representing the effect of the Earth's oblateness [unitless].

    Returns:
    tuple: ax, ay, az -> perturbation acceleration vector [m/s^2].
    """
    x, y, z = r_eci_m
    r_norm_m = np.linalg.norm(r_eci_m)

    if r_norm_m <= 0:
        raise ValueError("Position vector must be nonzero.")

    z2_over_r2 = (z**2) / (r_norm_m**2)
    factor = - 3 / 2 * (mu_m3_s2 * j2 * r_eq_m**2) / (r_norm_m**5)

    ax = factor * x * (1 - 5 * z2_over_r2)
    ay = factor * y * (1 - 5 * z2_over_r2)
    az = factor * z * (3 - 5 * z2_over_r2)

    return (ax, ay, az)
