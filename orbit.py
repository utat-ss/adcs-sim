# Handle any calculations relating to orbital mechanics

from pydantic import BaseModel, field_validator, model_validator, ValidationInfo
import numpy as np
from constants import G_m3_kgs2, M_kg, EARTH_MU_m3_s2
from collections.abc import Callable
from datetime import datetime, timezone, timedelta
from scipy.integrate import solve_ivp
import environment as env

G = G_m3_kgs2
M = M_kg
mu = EARTH_MU_m3_s2
e = np.e

class simulation_config():
    """
    Configuration for the simulation.
    """
    t0: datetime # Simulation start time (UTC)
    tf: datetime # Simulation end time (UTC)
    time_steps: int = 1000         # Number of time steps
    propagator_method: str = "cowell"  # Propagator to use: "cowell", "encke", or "sgp4"
    x0: np.ndarray = np.array([0., 0., 0., 0., 0., 0.]) # Initial state vector (6x1) (must be in ECI (for now))
    drag: bool = False # Whether to include drag in the simulation
    J2: bool = False # Whether to include J2 perturbation in the simulation
    
    def __init__(self, t0: datetime, tf: datetime, time_steps: int = 1000, propagator_method: str = "cowell", x0: np.ndarray = np.array([0., 0., 0., 0., 0., 0.]), drag: bool = False, J2: bool = False):
        self.t0 = t0
        self.tf = tf
        self.time_steps = time_steps
        self.propagator_method = propagator_method
        self.x0 = x0
        self.drag = drag
        self.J2 = J2

class KeplerianElements(BaseModel):
    """
    Set of Keplerian Elements for a specified orbit.
    """
    a_km: float     # semi-major axis [km]
    e: float        # eccentricity
    i_rad: float    # inclination [rad]
    Om_rad: float   # right ascension of the ascending node [rad]
    om_rad: float   # argument of periapsis [rad]

    @field_validator("i_rad", "Om_rad", "om_rad")
    @classmethod
    def angle_in_range(cls, v: float, info: ValidationInfo):
        """
        Check if angular values are within range 0-360 degrees. Does not stop values outside this range or wrap them, but
        users are encouraged to ensure values are wrapped.
        """
        if v < 0 or v >= 2 * np.pi:
            print(f"Warning: angular value outside range. Should be 0 <= {info.field_name} < 2pi rad, got {v} rad.")
        return v

    @field_validator("e")
    @classmethod
    def eccentricity_positive(cls, v: float):
        """
        Ensures eccentricity value is positive.
        """
        if v < 0:
            raise ValueError(f"Eccentricity must be greater than or equal to 0. Got {v}.")
        return v

    def as_array(self):
        return np.array([self.a_km, self.e, self.i_rad, self.Om_rad, self.om_rad])

class EquinoctialElements(BaseModel):
    """
    Set of Equinoctial Elements for a specified orbit.
    """
    a_km: float     # semi-major axis [km]
    h: float        # eccentricity vector y {e sin(Om + om)}
    k: float        # eccentricity vector x {e cos(Om + om)}
    p: float        # orientation vector y {tan(i/2) sin(Om)}
    q: float        # orientation vector x {tan(i/2) cos(Om)}

    def as_array(self):
        return np.array([self.a_km, self.h, self.k, self.p, self.q])

class ModifiedEquinoctialElements(BaseModel):
    """
    Set of Modified Equinoctial Elements for a specified orbit.
    """
    p_km: float     # semilatus rectum {a (1 - e^2)} [km]
    f: float        # eccentricity vector x {e cos(om + Om)}
    g: float        # eccentricity vector y {e sin(om + Om)}
    h: float        # orientation vector y {tan(i/2) sin(Om)}
    k: float        # orientation vector x {tan(i/2) cos(Om)}

    @field_validator("p_km")
    @classmethod
    def semilatus_rectum_positive(cls, v: float):
        """
        Ensures the semilatus rectum is strictly positive.
        """
        if v <= 0:
            raise ValueError(f"Semilatus rectum must be strictly greater than 0. Got {v}.")
        return v

    def as_array(self):
        return np.array([self.p_km, self.f, self.g, self.h, self.k])

def keplerian2equinoctial(kep: KeplerianElements) -> EquinoctialElements:
    """
    Convert Keplerian Elements to Equinoctial Elements.
    """
    pericenter_rad = kep.Om_rad + kep.om_rad
    half_inc_rad = kep.i_rad / 2

    h = kep.e * np.sin(pericenter_rad)
    k = kep.e * np.cos(pericenter_rad)
    p = np.tan(half_inc_rad) * np.sin(kep.Om_rad)
    q = np.tan(half_inc_rad) * np.cos(kep.Om_rad)
    
    eq = EquinoctialElements(a_km = kep.a_km,
                             h = h,
                             k = k,
                             p = p,
                             q = q)
    return eq

def keplerian2mee(kep: KeplerianElements) -> ModifiedEquinoctialElements:
    """
    Convert Keplerian Elements to Modified Equinoctial Elements.
    """
    pericenter_rad = kep.Om_rad + kep.om_rad
    half_inc_rad = kep.i_rad / 2

    p_km = kep.a_km * (1 - kep.e**2)
    f = kep.e * np.cos(pericenter_rad)
    g = kep.e * np.sin(pericenter_rad)
    h = np.tan(half_inc_rad) * np.sin(kep.Om_rad)
    k = np.tan(half_inc_rad) * np.cos(kep.Om_rad)

    me = ModifiedEquinoctialElements(p_km = p_km,
                                     f = f,
                                     g = g,
                                     h = h,
                                     k = k)
    return me

def equinoctial2keplerian(eq: EquinoctialElements) -> KeplerianElements:
    """
    Convert Equinoctial elements to Keplerian Elements.
    """
    e = np.sqrt(eq.h**2 + eq.k**2)
    i_rad = 2 * np.arctan(np.sqrt(eq.p**2 + eq.q**2))
    Om_rad = np.atan2(eq.p, eq.q)
    om_rad = np.atan2(eq.h, eq.k) - Om_rad

    kep = KeplerianElements(a_km = eq.a_km,
                            e = e,
                            i_rad = i_rad,
                            Om_rad = Om_rad,
                            om_rad = om_rad)

def mee2keplerian(me: ModifiedEquinoctialElements) -> KeplerianElements:
    """
    Convert Modified Equinoctial Elements to Keplerian Elements.
    """
    e = np.sqrt(me.f**2 + me.g**2)
    a_km = me.p_km / (1 - e**2)
    i_rad = 2 * np.arctan(np.sqrt(me.h**2 + me.k**2))
    Om_rad = np.atan2(me.k, me.h)
    om_rad = np.atan2(me.g, me.f) - Om_rad

    kep = KeplerianElements(a_km = a_km,
                            e = e,
                            i_rad = i_rad,
                            Om_rad = Om_rad,
                            om_rad = om_rad)
    return kep

def equinoctial2mee(eq: EquinoctialElements) -> ModifiedEquinoctialElements:
    """
    Convert Equinoctial Elements to Modified Equinoctial Elements.
    """
    e = np.sqrt(eq.h**2 + eq.k**2)
    p_km = eq.a_km * (1 - e**2)

    me = ModifiedEquinoctialElements(p_km = p_km,
                                     f = eq.h,
                                     g = eq.k,
                                     h = eq.p,
                                     k = eq.q)
    return me

def mee2equinoctial(me: ModifiedEquinoctialElements) -> EquinoctialElements:
    """
    Convert Modified Equinoctial Elements to Equinoctial Elements.
    """
    e = np.sqrt(me.f**2 + me.g**2)
    a_km = me.p_km / (1 - e**2)

    eq = EquinoctialElements(a_km = a_km,
                             h = me.f,
                             k = me.g,
                             p = me.h,
                             q = me.k)
    return eq

def true_anom2radius(nu_rad: float, a_km:float, e: float) -> float:
    """
    Calculate the orbit radius at a given true anomaly.

    Arguments:
    nu_rad:     [float] True anomaly.
    a_km:       [float] Semi-major axis.
    e:          [float] Eccentricity.

    Returns:
    r_km:       [float] Orbit radius.
    """
    r_km = a_km * (1 - e**2) / (1 + e * np.cos(nu_rad))
    return r_km

def get_orbit_period(a_km: float, mu_km3_s2: float) -> float:
    """
    Calculate an orbit's period given the semi-major axis around some primary body.

    Arguments:
    a_km:       [float] Semi-major axis.
    mu_km3_s2:  [float] Gravitational parameter of the primary body.

    Returns:
    T_s:        [float] Orbit period.
    """
    T_s = 2 * np.pi * np.sqrt(a_km**3 / mu_km3_s2)
    return T_s

def period2mean_motion(T_s: float) -> float:
    """
    Calculate the mean motion given an orbital period.

    Arguments:
    T_s:        [float] Orbit period.

    Returns:
    n_rad_s:    [float] Mean motion.
    """
    n_rad_s = 2 * np.pi / T_s
    return n_rad_s

def time2mean_anom(n_rad_s: float, dt_s: float) -> float:
    """
    Calculate the mean anomaly for a given time since passage of periapsis.

    Arguments:
    n_rad_s:    [float] Mean motion.
    dt_s:       [float] Time since passage of periapsis. Should be less than orbit period.

    Returns:
    M_rad:      [float] Mean anomaly.
    """
    M_rad = n_rad_s * dt_s
    return M_rad

def true_anom2ecc_anom(e: float, nu_rad: float) -> float:
    """
    Calculate the eccentric anomaly given the true anomaly.

    Arguments:
    e:      [float] Eccentricity.
    nu_rad: [float] True anomaly.

    Returns:
    E_rad:  [float] Eccentric anomaly.
    """
    E_rad = 2 * np.arctan(np.sqrt((1 - e)/(1 + e)) * np.tan(nu_rad / 2))
    return E_rad

def ecc_anom2mean_anom(E_rad: float, e: float) -> float:
    """
    Calculate the mean anomaly given the eccentric anomaly.

    Arguments:
    E_rad:  [float] Eccentric anomaly.
    e:      [float] Eccentricity.

    Returns:
    M_rad:  [float] Mean anomaly.
    """
    M_rad = E_rad - e * np.sin(E_rad)
    return M_rad

def ecc_anom2true_anom(E_rad: float, e: float) -> float:
    """
    Calculate the true anomaly given the eccentric anomaly.

    Arguments:
    E_rad:  [float] Eccentric anomaly.
    e:      [float] Eccentricity.

    Returns:
    nu_rad: [float] True anomaly.
    """
    nu_rad = 2 * np.arctan(np.sqrt((1 - e)/(1 + e)) * np.tan(E_rad / 2))
    return nu_rad

def mean_anom2ecc_anom(M_rad: float) -> float:
    """
    Calculate the eccentric anomaly given the mean anomaly.

    Arguments:
    M_rad:  [float] Mean anomaly.

    Returns:
    E_rad:  [float] Eccentric anomaly.
    """
    # TODO: requires root finder like Newton-Raphson; should be done in a separate module
    raise NotImplementedError("Mean anomaly to eccentric anomaly not implemented pending root finding tools")

def time2true_anom(a_km: float, e: float, mu_km3_s2: float, dt_s: float) -> float:
    """
    Calculate the true anomaly given an orbit's shape parameters and the time since periapsis passage.

    Arguments:
    a_km:       [float] Semi-major axis of the orbit.
    e:          [float] Eccentricity of the orbit.
    mu_km3_s2:  [float] Gravitational parameter of the primary body.
    dt_s:       [float] Time since passage of periapsis. Should be less than the orbital period.

    Returns:
    nu_rad:     [float] True anomaly at the specified time.
    """
    T_s = get_orbit_period(a_km, mu_km3_s2)
    n_rad_s = period2mean_motion(T_s)
    M_rad = time2mean_anom(n_rad_s, dt_s)
    E_rad = mean_anom2ecc_anom(M_rad)
    nu_rad = ecc_anom2true_anom(E_rad, e)
    return nu_rad

def epoch2time_since_periapsis(
        target_epoch_j2000_d: float,
        reference_epoch_j2000_d: float,
        kep_elements: KeplerianElements,
        reference_nu_rad: float,
        mu_km3_s2: float) -> float:
    """
    Convert target epoch time to time since the most recent passage of periapsis.

    Arguments:
    target_epoch_j2000_d:      [float] Target epoch, in days since J2000.
    reference_epoch_j2000_d:   [float] Epoch where the orbital state is known, in days since J2000.
    kep_elements:              [KeplerianElements] Orbit elements at the reference epoch.
    reference_nu_rad:          [float] True anomaly at the reference epoch.
    mu_km3_s2:                 [float] Gravitational parameter of the primary body.

    Returns:
    target_dt_s:               [float] Time since most recent passage of periapsis.
    """
    T_s = get_orbit_period(kep_elements.a_km, mu_km3_s2)
    n_rad_s = period2mean_motion(T_s)

    E_rad = true_anom2ecc_anom(kep_elements.e, reference_nu_rad)
    M_rad = ecc_anom2mean_anom(E_rad, kep_elements.e) % (2 * np.pi)

    reference_dt_s = M_rad / n_rad_s
    epoch_dt_s = (target_epoch_j2000_d - reference_epoch_j2000_d) * 86400
    target_dt_s = (reference_dt_s + epoch_dt_s) % T_s

    return target_dt_s

def get_ang_momentum(a_km: float, e: float, mu_km3_s2: float) -> float:
    """
    Calculate an orbit's angular momentum using shape parameters.

    Arguments:
    a_km:       [float] Semi-major axis of the orbit.
    e:          [float] Eccentricity of the orbit.
    mu_km3_s2:  [float] Gravitational parameter of the primary body.

    Returns:
    h_km2_s:    [float] Angular momentum of the orbit.
    """
    h_km2_s = np.sqrt(mu_km3_s2 * a_km * (1 - e**2))
    return h_km2_s

def keplerian2cartesian(kep: KeplerianElements, nu_rad: float, mu_km3_s2: float) -> np.ndarray:
    """
    Convert Keplerian elements and true anomaly to a cartesian state vector in the intertial frame.

    Arguments:
    kep:        [KeplerianElements] Set of Keplerian elements describing the whole orbit.
    nu_rad:     [float] True anomaly at the point of conversion.
    mu_km3_s2:  [float] Gravitational parameter of the primary body.

    Returns:
    x:          [np.ndarray] 6x1 Orbital state vector in cartesian inertial coordinates.
    """
    raise NotImplementedError("Keplerian elements to cartesian state vector not implemented pending rotation tools")

    # State vector in perifocal frame
    h_km2_s = get_ang_momentum(kep.a_km, kep.e, mu_km3_s2)
    r_w_km = h_km2_s**2 / mu_km3_s2 / (1 + kep.e * np.cos(nu_rad)) * np.array((np.cos(nu_rad), np.sin(nu_rad), 0))
    v_w_km_s = mu_km3_s2 / h_km2_s * np.array((-np.sin(nu_rad), kep.e + np.cos(nu_rad), 0))

    # Rotate to ECI
    # TODO: rotation utils
    R = C3(-om_rad) @ C1(-i_rad) @ C3(-Om_rad)
    r_g_km = r_w_km @ R
    v_g_km_s = v_w_km_s @ R

    # Combine into single state vector
    x = np.hstack([r_g_km, v_g_km_s])

    return x

def cartesian2keplerian(x: np.ndarray, mu_km3_s2: float) -> tuple[KeplerianElements, float]:
    """
    Convert a cartesian state vector to Keplerian elements with true anomaly.

    Arguments:
    x:                  [np.ndarray] (6x1) Orbital state vector in cartesian inertial frame in km.
    mu_km3_s2:          [float] Gravitational parameter of the primary body.

    Returns:
    kep:                [KeplerianElements] Object containing the global Keplerian elements of the orbit.
    nu_rad:             [float] True anomaly corresponding to the position vector.
    """
    r_g_km = x[0:3]
    r_km = np.linalg.norm(r_g_km)
    v_g_km_s = x[3:]
    v_km_s = np.linalg.norm(v_g_km_s)

    v_r_km_s = np.dot(v_g_km_s, r_g_km / r_km)
    v_t_km_s = np.sqrt(v_g_km_s**2 - v_r_km_s**2)

    # Orbit angular momentum
    h_km2_s = np.cross(r_g_km, v_g_km_s)

    # Find semi-major axis
    a_km = (mu_km3_s2 * r_km) / (2 * mu_km3_s2 - r_km * v_km_s**2)

    # Inclination
    i_rad = np.arccos(h_km2_s[2] / h_km2_s)

    # Right ascension of the ascending node
    K = np.array((0, 0, 1))
    N = np.cross(K, h_km2_s)
    Om_rad = 2 * np.pi - np.arccos(N[0] / np.linalg.norm(N))

    # Eccentricity
    e_vec = np.cross(v_g_km_s, h_km2_s) / mu_km3_s2 - r_g_km / r_km
    e = np.linalg.norm(e_vec)

    # Argument of periapsis
    om_rad = 2 * np.pi - np.arccos(np.dot(N, e_vec) / (N * e))

    # True anomaly
    nu_rad = np.arccos(np.dot(r_g_km / r_km, e_vec / e))

    kep = KeplerianElements(a_km = a_km,
                            e = e,
                            i_rad = i_rad,
                            Om_rad = Om_rad,
                            om_rad = om_rad)

    return kep, nu_rad

def newton_method(x_n: float, func: Callable[[float], float], d_func: Callable[[float], float], 
                  tolerance: float = 1e-10, max_iter: int = 20) -> float:
    """
    Newton's method, root finder. Used to solve transcendental function.
    
    Arguments:
    x_n:         (float) nth guess of root of equation
    func:        The function to solve
    d_func:      Derivative of the function to solve
    tolerance:   How precise the answer should be
    max_iter:    Maximum number of times to iterate

    Output:
    x_new:  n+1th guess of root of equation
    """

    for _ in range(max_iter):
        f = func(x_n)
        df = d_func(x_n)
        x_new = x_n - (f / df)

        if abs(x_new - x_n) < tolerance:
            return x_new
            # no longer changing the estimation enough to be meaningful
        
        x_n = x_new # update x to next step
        
    return x_n


def kepler_motion(kep: KeplerianElements, t: float):
    """
    Unperturbed keplerian propagator. 
    Given an orbital state described by x0, recorded at t0, return the propagated orbital state at time t.

    Arguments:
    kep:    KeplerianElements that describe the orbit. (semi-major axis [km], eccentricity, inclination [rad], right ascension of the ascending node [rad], argument of periapsis [rad])
    t:      (float) Time since periapsis.

    Output:
    r_osc_mag:     (float) Distance from central body of orbit to the spacecraft
    """
    a = kep.a_km
    n = np.sqrt(mu/a**3) # mean motion
    M = n*(t) 
    # in the normal formula: tp = time of periapsis, t = current time, and M = n*(t-tp)
    # but here the t is already current time - time of periapsis, so we pass that in

    E = newton_method(M, 
                      lambda E: E - e*np.sin(E) - M, # the equation we equate to 0 and are solving for
                      lambda E: 1 - e*np.cos(E)      # the derivative of equation we are solving for
                      )


    v = 2 * np.arctan2(np.sqrt(1 + e) * np.sin(E / 2),
                       np.sqrt(1 - e) * np.cos(E / 2)
                       ) # true anomaly
    
    r_osc_mag = a * (1-e**2) / (1+e*np.cos(v))

    return r_osc_mag
    


def mee_motion(me: ModifiedEquinoctialElements, L_rad: float, p_rsw_m2_s2: np.ndarray, mu_km3_s2: float) -> np.ndarray:
    """
    Given an orbital state described by MEEs and any perturbations in the Radial-Cross-track-Along-track frame (RSW),
    determine the time rate of change of these elements.

    Arguments:
    me:             [ModifiedEquinoctialElements] Modified equinoctial elements describing the orbit.
    L_rad:          [float] True longitude {Om + om + nu}
    p_rsw_m2_s2:    [np.ndarray] (3x1) Vector of summed perturbing accelerations in the RSW frame.
    mu_km3_s2:      [float] Gravitational parameter of the primary body.
    """
    
    # Auxiliary variables
    qx = 1 + me.f * np.cos(L_rad) + me.g * np.sin(L_rad)
    s2 = 1 + me.h**2 + me.k**2
    p_mu = np.sqrt(me.p_km / mu_km3_s2)

    # Equations of motion
    pdot_km_s = p_mu * 2 * (me.p_km / qx) * p_rsw_m2_s2[1]

    fdot__s = p_mu * (np.sin(L_rad) * p_rsw_m2_s2[0])\
            + (1 / qx) * ((qx + 1) * np.cos(L_rad) + me.f) * p_rsw_m2_s2[1]\
            - (me.g / qx) * (me.h * np.sin(L_rad) - me.k * np.cos(L_rad) * p_rsw_m2_s2[2])

    gdot__s = p_mu * (-np.cos(L_rad) * p_rsw_m2_s2[0])\
            + (1 / qx) * ((qx + 1) * np.sin(L_rad) + me.g) * p_rsw_m2_s2[1]\
            + (me.f / qx) * (me.h * np.sin(L_rad) - me.k * np.cos(L_rad) * p_rsw_m2_s2[2])

    hdot__s = p_mu * ((s2 * np.cos(L_rad)) / (2 * qx)) * p_rsw_m2_s2[2]

    kdot__s = p_mu * ((s2 * np.sin(L_rad)) / (2 * qx)) * p_rsw_m2_s2[2]

    Ldot_rad_s = np.sqrt(mu_km3_s2 * me.p_km) * (qx / me.p_km)**2\
            + p_mu * (1 / qx) * (me.h * np.sin(L_rad) - me.k * np.cos(L_rad)) * p_rsw_m2_s2[2]

    medot = np.array([pdot_km_s, fdot__s, gdot__s, hdot__s, kdot__s, Ldot_rad_s])

    return medot


def cowell_motion(x: np.ndarray, add_drag: bool, add_J2: bool) -> np.ndarray:
    """
    Calculate the orbital motion of a Cartesian state using Cowell's method.

    Arguments:
    x:      (np.ndarray) (6,) Orbital state vector. (x, y, z, v_x, v_y, v_z) in meters
    add_drag:   (bool) Whether to include atmospheric drag.
    add_J2:     (bool) Whether to include J2 perturbation.

    Returns:
    xdot:   (np.ndarray) (6,) Orbit motion.
    """
    dr = x[3:6] # is shape (3,)

    r_vec = x[0:3] # is shape (3,)
    r_mag = np.linalg.norm(r_vec) # magnitude of r vector
    p_m_s2 = np.array([0., 0., 0.]) # initialize perturbing acceleration vector
    if add_J2:
        p_m_s2 += env.j2_acceleration_m_s2(r_vec)
    if add_drag:
        air_velocity_eci_m_s = env.calc_atm_velocity_m_s(r_vec, np.array([0,0,7.292115*10**(-5)])) # should be replaced with more accurate, varying angular velocity value later
        air_density = env.approximate_atmospheric_density_kg_m3(r_vec) 
        p_m_s2 += env.aerodynamic_drag_perturbation_m_s2(dr, air_velocity_eci_m_s, air_density, drag_coeff=2.2, area_m_2=0.03, mass_kg=5.0) # may wanna include the parameters used to include drag in our satellite configuration file

    dv = (-mu*r_vec/(r_mag)**3)+p_m_s2
    
    xdot = np.concatenate((dr, dv))  # is shape (6,)
    return xdot


def encke_motion(t: float, x_ref: np.ndarray, delta_x: np.ndarray, add_drag: bool, add_J2: bool) -> np.ndarray:
    """
    Calculate the orbital motion of a Cartesian state using Encke's method.

    Arguments:
    r: (np.ndarray) (6,) (r, v).
    x: (np.ndarray) (6,) (delta_r, delta_r_dot) (deviation).
    p_m_s2: (np.ndarray) (3,) Perturbing accelerations.

    Returns:
    xdot:   (np.ndarray) (6,)  (delta_r, delta_r_dot) (deviation's derivative).
    """
    r_ref = x_ref[0:3]
    v_ref = x_ref[3:6]
    r_mag = np.linalg.norm(r_ref) # magnitude of r vector

    delta_r_dot = delta_x[3:6] # (3,)

    delta_r = delta_x[0:3] # (3,)

    q = np.dot(delta_r, (delta_r+2*r_ref)/np.linalg.norm(r_ref)**2)
    fq = q*((q**2+3*q+3)/((1+q)**1.5+1))
    a = -mu/r_mag**3*(delta_r-fq*(r_ref+delta_r))
    
    p_m_s2 = np.array([0., 0., 0.]) # initialize perturbing acceleration vector
    if add_J2:
        p_m_s2 += env.j2_acceleration_m_s2(r_ref+delta_r)
    if add_drag:
        air_velocity_eci_m_s = env.calc_atm_velocity_m_s(r_ref+delta_r, np.array([0,0,7.292115*10**(-5)])) # should be replaced with more accurate, varying angular velocity value later
        air_density = 1.0e-15#env.approximate_atmospheric_density_kg_m3(r_ref+delta_r)
        p_m_s2 += env.aerodynamic_drag_perturbation_m_s2(v_ref, air_velocity_eci_m_s, air_density, drag_coeff=2.2, area_m_2=0.03, mass_kg=5.0) #  may wanna include the parameters used to include drag in our satellite configuration file
    
    delta_r_dot_dot = a + p_m_s2

    xdot = np.concatenate((delta_r_dot, delta_r_dot_dot)) # (6,)

    return xdot

def stumpff_C(z):
    if z > 1e-8:
        sz = np.sqrt(z)
        return (1.0 - np.cos(sz)) / z

    elif z < -1e-8:
        sz = np.sqrt(-z)
        return (np.cosh(sz) - 1.0) / (-z)

    else:
        return (
            1.0 / 2.0
            - z / 24.0
            + z**2 / 720.0
            - z**3 / 40320.0
        )


def stumpff_S(z):
    if z > 1e-8:
        sz = np.sqrt(z)
        return (sz - np.sin(sz)) / sz**3

    elif z < -1e-8:
        sz = np.sqrt(-z)
        return (np.sinh(sz) - sz) / sz**3

    else:
        return (
            1.0 / 6.0
            - z / 120.0
            + z**2 / 5040.0
            - z**3 / 362880.0
        )


def kepler_cartesian_motion(
    x0_m,
    t,
    mu_m3_s2=mu,
    tolerance=1e-7,
    max_iter=100,
):
    """
    Propagate a Cartesian state under unperturbed two-body motion.

    Parameters
    ----------
    x0_m : np.ndarray, shape (6,)
        [x, y, z, vx, vy, vz]
        Position in m, velocity in m/s.

    t : float
        Time since initial state [s].

    mu_m3_s2 : float
        Gravitational parameter [m^3/s^2].

    Returns
    -------
    x : np.ndarray, shape (6,)
        Propagated Cartesian state [m, m/s].
    """

    x0_m = np.asarray(x0_m, dtype=float)

    if x0_m.shape != (6,):
        raise ValueError(
            f"x0_m must have shape (6,), got {x0_m.shape}"
        )

    r0_vec = x0_m[:3]
    v0_vec = x0_m[3:]

    r0 = np.linalg.norm(r0_vec)
    v0 = np.linalg.norm(v0_vec)

    if r0 == 0:
        raise ValueError("Initial position cannot be zero.")

    if t == 0:
        return x0_m.copy()

    sqrt_mu = np.sqrt(mu_m3_s2)

    # Initial radial velocity
    vr0 = np.dot(r0_vec, v0_vec) / r0

    # Reciprocal semi-major axis
    alpha = (
        2.0 / r0
        - v0**2 / mu_m3_s2
    )

    # Initial guess for universal anomaly.
    # This is especially appropriate for the elliptic Earth orbits
    # being tested here.
    if alpha > 1e-12:
        chi = sqrt_mu * alpha * t
    else:
        chi = sqrt_mu * t / r0

    # Solve universal Kepler equation
    for _ in range(max_iter):

        z = alpha * chi**2

        C = stumpff_C(z)
        S = stumpff_S(z)

        F = (
            (r0 * vr0 / sqrt_mu)
            * chi**2
            * C

            + (1.0 - alpha * r0)
            * chi**3
            * S

            + r0 * chi

            - sqrt_mu * t
        )

        dF = (
            (r0 * vr0 / sqrt_mu)
            * chi
            * (1.0 - z * S)

            + (1.0 - alpha * r0)
            * chi**2
            * C

            + r0
        )

        delta_chi = F / dF
        chi -= delta_chi

        if abs(delta_chi) < tolerance:
            break

    else:
        raise RuntimeError(
            "Universal-variable Kepler solver did not converge."
        )

    # Recalculate at converged chi
    z = alpha * chi**2
    C = stumpff_C(z)
    S = stumpff_S(z)

    # Lagrange f, g coefficients
    f = 1.0 - chi**2 / r0 * C

    g = (
        t
        - chi**3 / sqrt_mu * S
    )

    r_vec = (
        f * r0_vec
        + g * v0_vec
    )

    r = np.linalg.norm(r_vec)

    # Lagrange f-dot, g-dot
    f_dot = (
        sqrt_mu
        / (r * r0)
        * (
            alpha * chi**3 * S
            - chi
        )
    )

    g_dot = (
        1.0
        - chi**2 / r * C
    )

    v_vec = (
        f_dot * r0_vec
        + g_dot * v0_vec
    )

    return np.concatenate((r_vec, v_vec))


def propagate_orbit(config: simulation_config):
    """
    Propagate an orbit from an initial state using the parameters given in config.

    Arguments:
    config:     (simulation_config) Configuration for the simulation.

    Returns:
    x:         (np.ndarray) (time_steps, 6) Array of orbital states at each time step.
    """
    if config.propagator_method == "cowell":
        t_span = (0.0, (config.tf - config.t0).total_seconds())
        results = solve_ivp(fun=lambda t, x: cowell_motion(x, add_drag=config.drag, add_J2=config.J2),
                         t_span = (0.0, (config.tf - config.t0).total_seconds()),
                         y0=config.x0,
                         t_eval=np.linspace(t_span[0], t_span[1], config.time_steps),
                         rtol=1e-10, atol=1e-10, method='RK45')
        # print(np.linspace(config.t0.timestamp(), config.tf.timestamp(), config.time_steps))
        # print(results.t)
        return results.y
    elif config.propagator_method == "encke":
        duration_s = (config.tf - config.t0).total_seconds()

        t_span = (0.0, duration_s)

        t_eval = np.linspace(
            0.0,
            duration_s,
            config.time_steps
        )

        # Encke integrates DEVIATION from the reference orbit.
        delta_x0 = np.zeros(6)

        def rhs(t, delta_x):
            x_ref = kepler_cartesian_motion(
                config.x0,
                t,
            )
            return encke_motion(
                t=t,
                x_ref=x_ref,
                delta_x=delta_x,
                add_drag=config.drag,
                add_J2=config.J2,
            )

        results = solve_ivp(
            fun=rhs,
            t_span=t_span,
            y0=delta_x0,
            t_eval=t_eval,
            rtol=1e-10,
            atol=1e-10,
            method="RK45",
        )

        delta_x = results.y

        # Reconstruct reference orbit at requested times
        x_ref = np.column_stack([
            kepler_cartesian_motion(config.x0, t)
            for t in t_eval
        ])

        # Actual orbit = reference + deviation
        return x_ref + delta_x
        

def propagate_sgp4(tle: str, t: float):
    """
    Propagate an orbit from an initial state given by a TLE using SGP4.

    Arguments:
    tle:    (str) Two Line Element set describing the orbital state at a specified epoch.
    t:      (float) Time to retrieve orbital state vector.

    Returns:
    x:      (np.ndarray) Orbital state at specified time.
    """
    #TODO: Appropriate time specification (time since periapsis? epoch time?)
    #TODO: SGP4 implementation
    x = np.array([0., 0., 0., 0., 0., 0.])
    return x

if __name__ == "__main__":
    r0 = 7000000
    v0 = np.sqrt(mu/r0)
    x0 = np.array([r0, 0, 0, 0, v0, 0])

    t0 = datetime(2026, 8, 14, 0, 0, 0, tzinfo=timezone.utc)
    T = 2*np.pi*np.sqrt(r0**3/mu)
    tf = t0+timedelta(seconds=T)

    config = simulation_config( 
        t0=t0,
        tf=tf,
        time_steps=100,
        propagator_method="encke",
        x0=x0,
        drag=False,
        J2=False,
    )

    result = propagate_orbit(config)

    print(result.shape)
    print("Initial:")
    print(result[:, 0])

    print("Final:")
    print(result[:, -1])
    
    print(result)
