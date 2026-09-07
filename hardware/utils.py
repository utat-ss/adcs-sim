import math


def haversine(angle_rad: float) -> float:
    """
    Compute the haversine of an angle.

    Arguments:
        angle_rad: (float) Angle in radians.

    Returns:
        (float) Haversine value, equal to sin(angle_rad / 2) ** 2.
    """

    return (math.sin(angle_rad / 2)) ** 2


def great_circle_distance_geocentric_lla(
    lat_1_deg: float,
    long_1_deg: float,
    lat_2_deg: float,
    long_2_deg: float,
    center_dist_m: float,
) -> float:
    """
    Compute distance along a great circle between two points equidistant from Earth's center, where each point's location is given by a set of geocentric latitude-longitude coordinates.

    Arguments:
        lat_1_deg: (float) GEOCENTRIC latitude coordinate of the first point [degrees].
        long_1_deg: (float) Longitude coordinate of the first point [degrees].
        lat_2_deg: (float) GEOCENTRIC latitude coordinate of the second point [degrees].
        long_2_deg: (float) Longitude coordinate of the second point [degrees].
        center_dist_m: (float) Distance of both points from the Earth's center [meters].

    Returns:
        (float) Distance along a great circle between the pairs of coordinates given [meters].
    """

    delta_lat_rad = math.radians(lat_2_deg - lat_1_deg)
    delta_long_rad = math.radians(long_2_deg - long_1_deg)

    hav_theta = (
        haversine(delta_lat_rad)
        + math.cos(math.radians(lat_1_deg))
        * math.cos(math.radians(lat_2_deg))
        * haversine(delta_long_rad)
    )

    return 2 * math.asin(hav_theta**0.5) * center_dist_m


def euclidean_distance_geocentric_lla(
    lat_1_deg: float,
    long_1_deg: float,
    center_dist_1_m: float,
    lat_2_deg: float,
    long_2_deg: float,
    center_dist_2_m: float,
) -> float:
    """
    Compute Euclidean distance between two points, where each point's location is given by a set of geocentric latitude-longitude coordinates, and a distance from the Earth's center.

    Arguments:
        lat_1_deg: (float) GEOCENTRIC latitude coordinate of the first point [degrees].
        long_1_deg: (float) Longitude coordinate of the first point [degrees].
        center_dist_1_m: (float) Distance of the first point from the Earth's center [meters].
        lat_2_deg: (float) GEOCENTRIC latitude coordinate of the second point [degrees].
        long_2_deg: (float) Longitude coordinate of the second point [degrees].
        center_dist_2_m: (float) Distance of the second point from the Earth's center [meters].

    Returns:
        (float) Euclidean (straight-line) distance between the pairs of coordinates given [meters].
    """

    theta_1_rad = math.pi / 2 - math.radians(lat_1_deg)
    long_1_rad = math.radians(long_1_deg)
    theta_2_rad = math.pi / 2 - math.radians(lat_2_deg)
    long_2_rad = math.radians(long_2_deg)

    x1 = center_dist_1_m * math.sin(theta_1_rad) * math.cos(long_1_rad)
    y1 = center_dist_1_m * math.sin(theta_1_rad) * math.sin(long_1_rad)
    z1 = center_dist_1_m * math.cos(theta_1_rad)

    x2 = center_dist_2_m * math.sin(theta_2_rad) * math.cos(long_2_rad)
    y2 = center_dist_2_m * math.sin(theta_2_rad) * math.sin(long_2_rad)
    z2 = center_dist_2_m * math.cos(theta_2_rad)

    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)