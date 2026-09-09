import gnssanalysis
from datetime import datetime, timezone, timedelta
from orbit import simulation_config, propagate_orbit
import constants as const
from utils import conversions as conv
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import math
from astropy import coordinates as coord
from astropy import units as u
from astropy.time import Time

def ecef_to_eci(ecef_coords, time, velocity=None):
    pos_ecef = ecef_coords.tolist() * u.m
    state_rep = coord.CartesianRepresentation(pos_ecef)
    if type(velocity) == np.ndarray:
        vel_ecef = velocity.tolist() * (u.m/u.s)
        velrep = coord.CartesianDifferential(vel_ecef)
        state_rep = state_rep.with_differentials(velrep)
    itrs_coord = coord.ITRS(state_rep, obstime=time)
    gcrs_coord = itrs_coord.transform_to(coord.GCRS(obstime=time))

    if type(velocity) != np.ndarray:
        return np.array(
        [
            gcrs_coord.cartesian.x.to_value(u.m),
            gcrs_coord.cartesian.y.to_value(u.m),
            gcrs_coord.cartesian.z.to_value(u.m),
        ]
    )
    else:
        return np.array(
        [
            gcrs_coord.velocity.d_x.to_value(u.m/u.s),
            gcrs_coord.velocity.d_y.to_value(u.m/u.s),
            gcrs_coord.velocity.d_z.to_value(u.m/u.s),
        ]
        )

def examine_sp3_data(J2bool, dragbool):
    sp3_file = gnssanalysis.gn_io.sp3.read_sp3("/Users/mikaelsharify-funk/Desktop/EngSci/ADCS Projects/adcs-sim/CAS_Orbit_GEO_20260828T000000_20260828T235959_1.1.0.sp3")

    x_vals = sp3_file[('EST','X')].tolist()
    y_vals = sp3_file[('EST','Y')].tolist()
    z_vals = sp3_file[('EST','Z')].tolist()
    t0 = datetime(2026, 8, 28, 0, 0, 18,tzinfo=timezone.utc)

    true_position_arr = ((np.array([x_vals, y_vals, z_vals])).transpose()) * 1000
    pos = ecef_to_eci(true_position_arr[0], t0)

    v0_m_s = ecef_to_eci(true_position_arr[0], t0, np.array([74924.910050, 4125.118670, 4424.235410])/10.)

    duration = 180 # in min
    steps_per_min = 60 # 1 Hz
    x0 = np.array([
        pos[0], pos[1], pos[2],
        v0_m_s[0], v0_m_s[1], v0_m_s[2]
    ])

    tf = t0 + timedelta(minutes=duration)
    config = simulation_config(
        t0=t0,
        tf=tf,
        time_steps=duration*steps_per_min+1,
        propagator_method="cowell",
        x0=x0,
        drag=dragbool,
        J2=J2bool
    )

    times = np.linspace(0, duration, duration * steps_per_min + 1) 

    raw_states = propagate_orbit(config)
    propagated_pos_data = np.transpose(raw_states[:3, :])

    true_pos_ecef = true_position_arr[:duration*steps_per_min + 1, :]

    true_pos_eci = []
    for true_pos_ind in range(len(true_pos_ecef)):
        true_pos_eci.append(ecef_to_eci(true_pos_ecef[true_pos_ind], t0 + timedelta(minutes=times[true_pos_ind])))
    true_pos_eci = np.asarray(true_pos_eci, dtype=float)

    pos_diff = true_pos_eci - propagated_pos_data
    normed_diffs = np.linalg.norm(pos_diff, axis=1)

    center_dist1 = np.linalg.norm(true_pos_eci, axis=1)
    center_dist2 = np.linalg.norm(propagated_pos_data, axis=1)
    alt_diffs = np.abs(center_dist1 - center_dist2)

    return times, normed_diffs, alt_diffs

# print(examine_sp3_data(True, False))
# Fields in CASSIOPE SP3 Data:

# MultiIndex([(  'EST',           'X'),
#             (  'EST',           'Y'),
#             (  'EST',           'Z'),
#             (  'EST',         'CLK'),
#             (  'STD',           'X'),
#             (  'STD',           'Y'),
#             (  'STD',           'Z'),
#             (  'STD',         'CLK'),
#             ('FLAGS', 'Clock_Event'),
#             ('FLAGS',  'Clock_Pred'),
#             ('FLAGS',    'Maneuver'),
#             ('FLAGS',  'Orbit_Pred')],
#            )