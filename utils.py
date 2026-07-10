import numpy as np


from __future__ import annotations

from datetime import datetime, timezone
from typing import Literal, Tuple

import numpy as np
import astropy.units as u
from astropy.time import Time
from astropy.coordinates import (
    GCRS,
    ITRS,
    TEME,
    EarthLocation,
    CartesianRepresentation,
)
from astropy.utils import iers


# Let astropy fetch recent IERS-A data if needed.
iers.conf.auto_download = True


ECIFrame = Literal["gcrs", "teme"]


def _time_utc(dt: datetime) -> Time:
    """
    Convert Python datetime to Astropy UTC Time.
    """
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    else:
        dt = dt.astimezone(timezone.utc)

    return Time(dt, scale="utc")


def eci_to_lla(
    r_eci_m: np.ndarray,
    dt: datetime,
    frame: ECIFrame = "gcrs",
) -> Tuple[float, float, float]:
    """
    Convert ECI position to geodetic latitude, longitude, altitude using Astropy frame transforms.

    Parameters
    ----------
    r_eci_m : array-like, shape (3,)
        ECI position in meters.
    dt : datetime
        UTC datetime.
    frame : {"gcrs", "teme"}
        ECI-like frame of r_eci_m.

    Returns
    -------
    lat_deg : float
        Geodetic latitude in degrees.
    lon_deg : float
        Longitude in degrees.
    alt_m : float
        Height above WGS84 ellipsoid in meters.
    """
    r_eci_m = np.asarray(r_eci_m, dtype=float)
    t = _time_utc(dt)

    rep = CartesianRepresentation(
        r_eci_m[0] * u.m,
        r_eci_m[1] * u.m,
        r_eci_m[2] * u.m,
    )

    if frame == "gcrs":
        eci_coord = GCRS(rep, obstime=t)
    elif frame == "teme":
        eci_coord = TEME(rep, obstime=t)
    else:
        raise ValueError("frame must be 'gcrs' or 'teme'")

    itrs = eci_coord.transform_to(ITRS(obstime=t))

    loc = EarthLocation.from_geocentric(
        itrs.x,
        itrs.y,
        itrs.z,
    )

    lon, lat, height = loc.to_geodetic()

    return (
        lat.to_value(u.deg),
        lon.to_value(u.deg),
        height.to_value(u.m),
    )


def lla_to_eci(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    dt: datetime,
    frame: ECIFrame = "gcrs",
) -> np.ndarray:
    """
    Convert geodetic latitude, longitude, altitude to ECI position using Astropy frame transforms.

    Parameters
    ----------
    lat_deg : float
        Geodetic latitude in degrees.
    lon_deg : float
        Longitude in degrees.
    alt_m : float
        Height above WGS84 ellipsoid in meters.
    dt : datetime
        UTC datetime.
    frame : {"gcrs", "teme"}
        Desired output ECI-like frame.

    Returns
    -------
    np.ndarray, shape (3,)
        ECI position in meters.
    """
    t = _time_utc(dt)

    loc = EarthLocation.from_geodetic(
        lon=lon_deg * u.deg,
        lat=lat_deg * u.deg,
        height=alt_m * u.m,
    )

    itrs = loc.get_itrs(obstime=t)

    if frame == "gcrs":
        eci = itrs.transform_to(GCRS(obstime=t))
    elif frame == "teme":
        eci = itrs.transform_to(TEME(obstime=t))
    else:
        raise ValueError("frame must be 'gcrs' or 'teme'")

    return np.array(
        [
            eci.cartesian.x.to_value(u.m),
            eci.cartesian.y.to_value(u.m),
            eci.cartesian.z.to_value(u.m),
        ]
    )

def normalize_quat(q: np.ndarray) -> np.ndarray:
    """
    Normalize quaternion q = [x, y, z, w].
    """
    q = np.asarray(q, dtype=float)

    if q.shape != (4,):
        raise ValueError(f"Quaternion must have shape (4,), got {q.shape}")

    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError("Quaternion has zero norm.")

    return q / norm

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Hamilton product q = q1 ⊗ q2.

    Both quaternions use [x, y, z, w].
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ], dtype=float)

def quat_from_rotvec(rotvec_rad: np.ndarray) -> np.ndarray:
    """
    Convert rotation vector to quaternion [x, y, z, w].

    rotvec_rad direction is the rotation axis.
    rotvec_rad magnitude is the rotation angle in radians.
    """
    rotvec_rad = np.asarray(rotvec_rad, dtype=float)

    if rotvec_rad.shape != (3,):
        raise ValueError(
            f"Rotation vector must have shape (3,), got {rotvec_rad.shape}"
        )

    angle = np.linalg.norm(rotvec_rad)

    if angle < 1e-15:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    axis = rotvec_rad / angle
    half_angle = 0.5 * angle

    xyz = axis * np.sin(half_angle)
    w = np.cos(half_angle)

    return np.array([xyz[0], xyz[1], xyz[2], w], dtype=float)


def euler_to_quat(
    roll: float,
    pitch: float,
    yaw: float,
    degrees: bool = False,
) -> np.ndarray:
    """
    Convert roll, pitch, yaw Euler angles to quaternion [x, y, z, w].

    Convention:
        R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    Parameters
    ----------
    roll : float
        Rotation about x-axis.
    pitch : float
        Rotation about y-axis.
    yaw : float
        Rotation about z-axis.
    degrees : bool
        If True, inputs are interpreted as degrees.

    Returns
    -------
    np.ndarray, shape (4,)
        Quaternion [x, y, z, w].
    """
    if degrees:
        roll, pitch, yaw = np.radians([roll, pitch, yaw])

    cr = np.cos(roll / 2.0)
    sr = np.sin(roll / 2.0)

    cp = np.cos(pitch / 2.0)
    sp = np.sin(pitch / 2.0)

    cy = np.cos(yaw / 2.0)
    sy = np.sin(yaw / 2.0)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)

    return q


def euler_to_rotmat(
    roll: float,
    pitch: float,
    yaw: float,
    degrees: bool = False,
) -> np.ndarray:
    """
    Convert roll, pitch, yaw Euler angles to a 3x3 rotation matrix.

    Convention:
        R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    Parameters
    ----------
    roll : float
        Rotation about x-axis.
    pitch : float
        Rotation about y-axis.
    yaw : float
        Rotation about z-axis.
    degrees : bool
        If True, inputs are interpreted as degrees.

    Returns
    -------
    np.ndarray, shape (3, 3)
        Rotation matrix.
    """
    if degrees:
        roll, pitch, yaw = np.radians([roll, pitch, yaw])

    cr = np.cos(roll)
    sr = np.sin(roll)

    cp = np.cos(pitch)
    sp = np.sin(pitch)

    cy = np.cos(yaw)
    sy = np.sin(yaw)

    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                  cp * cr],
    ])

    return R


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """
    Convert quaternion [x, y, z, w] to a 3x3 rotation matrix.

    Parameters
    ----------
    q : array-like, shape (4,)
        Quaternion in [x, y, z, w] convention.

    Returns
    -------
    np.ndarray, shape (3, 3)
        Rotation matrix.
    """
    x, y, z, w = np.asarray(q, dtype=float)
    norm = np.sqrt(x*x + y*y + z*z + w*w)

    if norm == 0:
        raise ValueError("Zero quaternion is not a valid rotation.")

    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),         2*(x*z + y*w)],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),         1 - 2*(x*x + y*y)],
    ])

    return R


def rot_x(theta_rad: float) -> np.ndarray:
    """
    Returns the rotation matrix around the X-axis (C1)

    Parameters
    ----------
    theta_rad : float
        Rotation angle in radians.

    Returns
    -------
    np.ndarray, shape (3, 3)
        Rotation matrix.
    """

    theta_rad = float(theta_rad)
    c = np.cos(theta_rad)
    s = np.sin(theta_rad)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c,   -s],
        [0.0, s,    c]
    ], dtype=float)


def rot_y(theta_rad: float) -> np.ndarray:
    """
    Returns the rotation matrix around the Y-axis (C2)

    Parameters
    ----------
    theta_rad : float
        Rotation angle in radians.

    Returns
    -------
    np.ndarray, shape (3, 3)
        Rotation matrix.
    """

    theta_rad = float(theta_rad)
    c = np.cos(theta_rad)
    s = np.sin(theta_rad)
    return np.array([
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c]
    ], dtype=float)


def rot_z(theta_rad: float) -> np.ndarray:
    """
    Returns the rotation matrix around the Z-axis (C3)

    Parameters
    ----------
    theta_rad : float
        Rotation angle in radians.

    Returns
    -------
    np.ndarray, shape (3, 3)
        Rotation matrix.
    """

    theta_rad = float(theta_rad)
    c = np.cos(theta_rad)
    s = np.sin(theta_rad)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=float)