from datetime import datetime
import ppigrf
from utils import spherical_to_cartesian

def get_magnetic_field(r: float, theta: float, phi: float, t: datetime, coord: str = 'spherical') -> tuple:
    """
    Calculate the magnetic field at a given position.
    Parameters
    r (float): The radial distance from the center of the Earth.
    theta (float): The polar angle in spherical coordinates.
    phi (float): The azimuthal angle in spherical coordinates.
    t (datetime): The time at which to calculate the magnetic field.
    coord (str): The coordinate system to use ('spherical' by default).
    """
    Br, Btheta, Bphi = ppigrf.igrf_gc(r, theta, phi, t)
    if coord == 'spherical':
        return Br, Btheta, Bphi
    elif coord == 'cartesian':
        return spherical_to_cartesian(Br, Btheta, Bphi) 