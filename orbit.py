# Handle any calculations relating to orbital mechanics


from dataclasses import dataclass

from pydantic import BaseModel, field_validator, model_validator, ValidationInfo
import numpy as np
from constants import G_m3_kgs2, M_kg, EARTH_MU_m3_s2
from collections.abc import Callable
from datetime import datetime, timezone, timedelta
from scipy.integrate import solve_ivp
import environment as env
from utils import conversions as conv
from dataclasses import dataclass, field
from datetime import datetime
from typing import Protocol
import numpy as np
from simulation import SimulationConfig, Spacecraft, PerturbationModel

G = G_m3_kgs2
M = M_kg
mu = EARTH_MU_m3_s2
mu_km3_s2 = mu / 1e9
e = np.e




class J2Perturbation(PerturbationModel):

    def acceleration(self, t, state, spacecraft):
        r = state[:3]

        return env.j2_acceleration_m_s2(r)

class DragPerturbation(PerturbationModel):

    def acceleration(self, t, state, spacecraft):
        r = state[:3]
        v = state[3:6]

        v_atm = env.calc_atmospheric_velocity_m_s(r, t)
        rho = env.atmospheric_density_kg_m3(r, t)

        return env.aerodynamic_drag_perturbation_m_s2(
            velocity_m_s=v,
            velocity_atm_m_s=v_atm,
            air_kg_m3=rho,
            drag_coeff=spacecraft.drag_coefficient,
            area_m_2=spacecraft.area_m2,
            mass_kg=spacecraft.mass_kg,
        )

def combine_perturbations(perturbations, t, state, spacecraft):
    a = np.zeros(3)
    for perturbation in perturbations:
        a+=perturbation.acceleration(t, state, spacecraft)
    return a

class CowellPropagator:

    def propagate(self, config, spacecraft):

        duration = (config.tf - config.t0).total_seconds()

        t_eval = np.linspace(
            0.0,
            duration,
            config.output_steps,
        )

        rhs = lambda t, x: cowell_motion(
            t,
            x,
            spacecraft,
            config.perturbations,
        )

        result = solve_ivp(
            rhs,
            (0.0, duration),
            config.x0,
            t_eval=t_eval,
            rtol=1e-10,
            atol=1e-10,
        )

        return result.t, result.y

class EnckePropagator:

    def propagate(self, config, spacecraft):

        duration = (config.tf - config.t0).total_seconds()

        t_eval = np.linspace(
            0.0,
            duration,
            config.output_steps,
        )

        delta_x0 = np.zeros(6)

        rhs = lambda t, dx: encke_motion(
            t,
            1000*kepler_motion(config.x0/1000, t),
            dx,
            config,
        )

        result = solve_ivp(
            rhs,
            (0.0, duration),
            delta_x0,
            t_eval=t_eval,
            rtol=1e-10,
            atol=1e-10,
        )
        print(result.t)
        # print("WTFFFFFFFFFFFFFFFFFFFFF")
        X_ref = np.column_stack([
            1000*kepler_motion(config.x0/1000, t)
            for t in result.t
        ])

        X = X_ref + result.y

        return result.t, X


    

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

    # State vector in perifocal frame
    h_km2_s = get_ang_momentum(kep.a_km, kep.e, mu_km3_s2)
    r_w_km = h_km2_s**2 / mu_km3_s2 / (1 + kep.e * np.cos(nu_rad)) * np.array((np.cos(nu_rad), np.sin(nu_rad), 0))
    v_w_km_s = mu_km3_s2 / h_km2_s * np.array((-np.sin(nu_rad), kep.e + np.cos(nu_rad), 0))

    # Rotate to ECI
    # TODO: rotation utils
    R = conv.rot_z(-kep.om_rad) @ conv.rot_x(-kep.i_rad) @ conv.rot_z(-kep.Om_rad)
    r_g_km = r_w_km @ R
    v_g_km_s = v_w_km_s @ R

    # Combine into single state vector
    x = np.hstack([r_g_km, v_g_km_s])

    return x

def cartesian2keplerian(
    x: np.ndarray,
    mu_km3_s2: float
) -> tuple[KeplerianElements, float]:

    r_vec = x[:3]
    v_vec = x[3:]

    r = np.linalg.norm(r_vec)
    v = np.linalg.norm(v_vec)

    # Angular momentum
    h_vec = np.cross(r_vec, v_vec)
    h = np.linalg.norm(h_vec)

    # Node vector
    k_hat = np.array([0.0, 0.0, 1.0])
    n_vec = np.cross(k_hat, h_vec)
    n = np.linalg.norm(n_vec)

    # Eccentricity vector
    e_vec = (
        np.cross(v_vec, h_vec) / mu_km3_s2
        - r_vec / r
    )
    e = np.linalg.norm(e_vec)

    # Semi-major axis
    a = 1.0 / (
        2.0 / r
        - v**2 / mu_km3_s2
    )

    # Inclination
    i = np.arccos(
        np.clip(h_vec[2] / h, -1.0, 1.0)
    )

    # RAAN
    Om = np.arctan2(
        n_vec[1],
        n_vec[0]
    ) % (2 * np.pi)

    # Argument of periapsis
    om = np.arctan2(
        np.dot(np.cross(n_vec, e_vec), h_vec)
        / (n * e * h),
        np.dot(n_vec, e_vec)
        / (n * e),
    ) % (2 * np.pi)

    # True anomaly
    nu = np.arctan2(
        np.dot(np.cross(e_vec, r_vec), h_vec)
        / (e * r * h),
        np.dot(e_vec, r_vec)
        / (e * r),
    ) % (2 * np.pi)

    kep = KeplerianElements(
        a_km=a,
        e=e,
        i_rad=i,
        Om_rad=Om,
        om_rad=om,
    )

    return kep, nu


def kepler_motion(x: np.ndarray, t: float):
    """
    Unperturbed keplerian propagator. 
    Given an orbital state described by x0, recorded at t0, return the propagated orbital state at time t.

    Arguments:
    x:      (np.ndarray) (6,) Orbital state vector in cartesian inertial frame in km.
    t:      (float) Time since periapsis.

    Output:
    r_osc_mag:     (float) Distance from central body of orbit to the spacecraft
    """
    kep, nu0_rad = cartesian2keplerian(x, mu_km3_s2)
    a = kep.a_km
    e = kep.e
    n = np.sqrt(mu_km3_s2/a**3) # mean motion

    # nu0 -> E0
    E0 = 2 * np.arctan2(
        np.sqrt(1 - e) * np.sin(nu0_rad / 2),
        np.sqrt(1 + e) * np.cos(nu0_rad / 2),
    )

    # E0 -> M0
    M0 = E0 - e * np.sin(E0)

    # Propagate mean anomaly
    M = M0 + n * t

    E = newton_method(M, 
                      lambda E: E - e*np.sin(E) - M, # the equation we equate to 0 and are solving for
                      lambda E: 1 - e*np.cos(E)      # the derivative of equation we are solving for
                      )


    v = 2 * np.arctan2(np.sqrt(1 + e) * np.sin(E / 2),
                       np.sqrt(1 - e) * np.cos(E / 2)
                       ) # true anomaly

    return keplerian2cartesian(kep, v, mu_km3_s2)


def cowell_motion(t: float, x: np.ndarray, spacecraft, perturbations) -> np.ndarray:
    """
    Calculate the orbital motion of a Cartesian state using Cowell's method.

    Arguments:
    x:      (np.ndarray) (6,) Orbital state vector. (x, y, z, v_x, v_y, v_z) in meters

    Returns:
    xdot:   (np.ndarray) (6,) Orbit motion.
    """
    dr = x[3:6] # is shape (3,)

    r_vec = x[0:3] # is shape (3,)
    r_mag = np.linalg.norm(r_vec) # magnitude of r vector

    p_m_s2 = combine_perturbations(perturbations, t, x, spacecraft)

    dv = (-mu*r_vec/(r_mag)**3)+p_m_s2
    
    xdot = np.concatenate((dr, dv))  # is shape (6,)
    return xdot


def encke_motion(t: float, x_ref: np.ndarray, delta_x: np.ndarray, config: SimulationConfig) -> np.ndarray:
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
    
    p_m_s2 = combine_perturbations(config.perturbations, t, 1000*kepler_motion(config.x0/1000, t), config.spacecraft)
    delta_r_dot_dot = a + p_m_s2

    xdot = np.concatenate((delta_r_dot, delta_r_dot_dot)) # (6,)

    return xdot


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
