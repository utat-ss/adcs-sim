import numpy as np
import matplotlib.pyplot as plt
import math

from hardware.sensors import sensors
from utils import conversions as conv, geometric_calculations as gc, quaternion_math as quat
import constants as const
from hardware import adcs

def get_meas_vect(alpha, beta, DCM):
    unit_x = np.array([1,0,0])
    alpha_vec = np.array([0,1,0]) * math.tan(np.deg2rad(alpha))
    beta_vec = np.array([0,0,1]) * math.tan(np.deg2rad(beta))
    vect = unit_x + alpha_vec + beta_vec
    # DCMinv = np.linalg.inv(DCM)
    return DCM @ vect

def get_res(): # np.identity(3),conv.rot_z(-math.pi/2), , 
    DCMs = [np.identity(3), conv.rot_z(math.pi/2)]#, conv.rot_z(math.pi), conv.rot_y(math.pi/2), conv.rot_y(-math.pi/2)]
    # print(DCMs)
    R = const.EARTH_RADIUS_EQ_m + 500000
    EARTH_TO_SUN_VEC = np.array([10**7, 5*10**7, 0.])

    angles = np.linspace(0, 2*math.pi, 100)

    FSS = sensors.VirtualFSS("/Users/mikaelsharify-funk/Desktop/EngSci/ADCS Projects/fss_15.json")

    positions = []
    for angle in angles:
        positions.append([R * math.cos(-angle), R * math.sin(-angle), 0])

    positions = np.asarray(positions, dtype=float)
    attitude_q = np.array([0,0,math.sin(math.pi/8),math.cos(math.pi/8)])

    res = []

    ADC = adcs.ADCS("id", np.identity(3))
    for pos in positions:
        earth_vec = -pos
        sun_vec = earth_vec + EARTH_TO_SUN_VEC
        # print(sun_vec)
        processed_meas = []
        for DCM in DCMs:
            meas = FSS.measure(attitude_q, sun_vec, ADC.eval_eclipse_heuristic(sun_vec, earth_vec)[0], DCM)
            # print(ADC.eval_eclipse_heuristic(sun_vec, earth_vec)[0], meas)
            vect = get_meas_vect(meas["alpha_deg"], meas["beta_deg"], conv.quat_to_rotmat(attitude_q) @ DCM )
            processed_meas.append(gc.angle_between_vectors(sun_vec, vect))
        res.append(processed_meas)

    res = np.asarray(res, dtype=float)

    return angles, res

# get_res()

# NEED TO FIX ERROR IN ROTMAT->QUAT

#conv.rot_z(-math.pi/2), conv.rot_z(math.pi/2), conv.rot_z(math.pi), conv.rot_y(math.pi/2), conv.rot_y(-math.pi/2)