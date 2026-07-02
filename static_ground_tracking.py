import matplotlib.pyplot as plt
import cartopy.crs as ccrs
import numpy as np
from orbit import KeplerianElements, get_ang_momentum, true_anom2radius

# Physical constants 
MU_EARTH_KM3_S2: float = 398600.4418  # Earth's gravitational parameter [km^3/s^2]
OMEGA_EARTH_RAD_S: float = 7.2921150e-5  # Earth rotation rate [rad/s]

# Visualization  
def plot_ground_track(
    lat_deg_list: list[float],
    lon_deg_list: list[float],
) -> None:
    """
    Satellite ground track on Mercator Projection.

    Parameters
    ----------
    lat_deg_list : list[float]
        Geodetic latitude values [deg].
    lon_deg_list : list[float]
        Longitude values in Earth-fixed frame [deg].
    """

    fig = plt.figure(figsize=(12, 6))
    ax = plt.axes(projection=ccrs.PlateCarree())

    ax.set_global()
    ax.stock_img()
    ax.coastlines()
    ax.gridlines(draw_labels=True)

    ax.scatter(
        lon_deg_list,
        lat_deg_list,
        color="red",
        s=5,
        transform=ccrs.PlateCarree(),
    )

    plt.title("Satellite Ground Track")
    plt.show()

def get_geo_coords(a_km = 6771.0, e = 0.001, i_rad = np.radians(51.6), 
                   Om_rad = 0.0, om_rad = 0.0) -> tuple[list[float], list[float]]:
    """
    Get geodetic latitude and longitude coordinates for a satellite ground track.
    
    TODO: ADD multi-orbit ground tracking
    TODO: CHG time step instead of true anomaly step when "mean_anom2ecc_anom" function is completed in orbit.py
    TODO: ADD perturbation effects (J2, drag, etc.) for more realistic ground tracks
    """
    # Define test orbit (default is ISS-like)
    orbit = KeplerianElements(
        a_km = a_km,                 # semi-major axis [km]
        e = e,                     # eccentricity 
        i_rad = i_rad,      # inclination [rad]
        Om_rad = Om_rad,                  # RAAN [rad] = 0 for static orbit
        om_rad = om_rad,                  # argument of periapsis [rad] = 0 for static orbit
    )

    # True anomaly sampling for one full orbit
    true_anom_rad_array: np.ndarray = np.linspace(0.0, 2 * np.pi, 360) #360 points for smooth ground track 
    d_true_anom_rad: float = true_anom_rad_array[1] - true_anom_rad_array[0]

    # Orbital angular momentum (needed for time mapping)
    h_km2_s: float = get_ang_momentum(
        orbit.a_km,
        orbit.e,
        MU_EARTH_KM3_S2,
    )

    lat_deg_list: list[float] = []
    lon_deg_list: list[float] = []

    # Simulation time state [s]
    elapsed_time_s: float = 0.0

    print("Computing state trajectory...")

    for true_anom_rad in true_anom_rad_array:
        argument_latitude_rad: float = orbit.om_rad + true_anom_rad

        # Latitude from orbital geometry [rad]
        lat_rad: float = np.arcsin(
            np.sin(orbit.i_rad) * np.sin(argument_latitude_rad)
        )

        # Longitude in inertial frame [rad]
        lon_eci_rad: float = np.arctan2(
            np.cos(orbit.i_rad) * np.sin(argument_latitude_rad),
            np.cos(argument_latitude_rad),
        )

        # Convert to Earth-fixed longitude by accounting for Earth rotation
        lon_ecef_rad: float = (
            lon_eci_rad
            + orbit.Om_rad
            - OMEGA_EARTH_RAD_S * elapsed_time_s
        )

        # Store results in degrees 
        lat_deg_list.append(np.degrees(lat_rad))

        lon_deg_wrapped: float = (
            (np.degrees(lon_ecef_rad) + 180.0) % 360.0 - 180.0
        )
        lon_deg_list.append(lon_deg_wrapped)

        # Convert anomaly step → time step using Kepler's second law
        radius_km: float = true_anom2radius(
            true_anom_rad,
            orbit.a_km,
            orbit.e,
        )

        time_step_s: float = (radius_km**2 / h_km2_s) * d_true_anom_rad
        elapsed_time_s += time_step_s

    return(lat_deg_list, lon_deg_list)

lat, lon = get_geo_coords()
plot_ground_track(lat, lon)
