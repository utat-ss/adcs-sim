import gnssanalysis
from datetime import datetime, timezone, timedelta
from orbit import simulation_config, propagate_orbit
import constants as const
from utils import conversions as conv
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import math

def examine_sp3_data(J2bool, dragbool):
    sp3_file = gnssanalysis.gn_io.sp3.read_sp3("/Users/mikaelsharify-funk/Desktop/EngSci/ADCS Projects/adcs-sim/CAS_Orbit_GEO_20260828T000000_20260828T235959_1.1.0.sp3")

    x_vals = sp3_file[('EST','X')].tolist()
    y_vals = sp3_file[('EST','Y')].tolist()
    z_vals = sp3_file[('EST','Z')].tolist()

    true_position_arr = ((np.array([x_vals, y_vals, z_vals])).transpose()) * 1000

    v0_dm_s = [74924.910050, 4125.118670, 4424.235410]

    duration = 180 # in min
    steps_per_min = 60 # 1 Hz
    x0 = np.array([
        true_position_arr[0][0], true_position_arr[0][1], true_position_arr[0][2],
        v0_dm_s[0]/10, v0_dm_s[1]/10, v0_dm_s[2]/10
    ])

    t0 = datetime(2026, 9, 4, 0, 0, 0,tzinfo=timezone.utc)
    tf = t0 + timedelta(minutes=duration)
    config = simulation_config(
        t0=t0,
        tf=tf,
        time_steps=duration*steps_per_min,
        propagator_method="cowell",
        x0=x0,
        drag=dragbool,
        J2=J2bool
    )

    raw_states = propagate_orbit(config)
    propagated_pos_data = np.transpose(raw_states[:3, :])

    true_pos = true_position_arr[1:(duration*steps_per_min+1), :]
    
    pos_diff = true_pos - propagated_pos_data
    normed_diffs = np.linalg.norm(pos_diff, axis=1)

    center_dist1 = np.linalg.norm(true_pos, axis=1)
    center_dist2 = np.linalg.norm(propagated_pos_data, axis=1)
    alt_diffs = np.abs(center_dist1 - center_dist2)

    times = np.linspace(1/steps_per_min, duration, duration * steps_per_min)

    return times, normed_diffs, alt_diffs


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