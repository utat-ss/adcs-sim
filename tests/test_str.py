import pytest
import math
import numpy as np

from ..hardware.sensors.sensors import VirtualSTR
from pathlib import Path

from ..utils import geometric_calculations as gc
from ..utils import quaternion_math as quat
from .. import constants as const

STR_CFG = Path("../hardware/sensors/icd/str/sagitta.json")

RAD_SUN = const.SUN_RADIUS_m
RAD_EARTH = const.EARTH_RADIUS_EQ_m
RAD_MOON = const.MOON_RADIUS_m

def test_loads_config():
    """
    Verify that STR parameters taken from the config file used are correct.
    """
    str = VirtualSTR(STR_CFG)
    
    assert str.type == "STR"
    assert str.model == "Sagitta"
    assert str.fov_rad == pytest.approx(0.4433136300)
    assert str.exclusion_rad == pytest.approx(1.3962634016)
    assert str.max_update_rate_hz == pytest.approx(10.0)
    assert str.max_body_rate_radhz == pytest.approx(0.0436332313)
    assert str.cov_rad2.shape == (3, 3)

def get_cone_edge_angles(str: VirtualSTR, q: np.ndarray, body_start: np.ndarray, body_end: np.ndarray, body_name: str, step_num: int) -> list:
    """
    Determine the angles (between a body's position vector and the STR boresight vector) at which
    the body given by body_name is just outside the STR's FOV/exclusion cone as the body moves
    from position body_start to position body_end in step_num increments.
    """

    body_inc = (body_end - body_start) / step_num

    num_of_inc = 0
    body_rates = np.array([0,0,0])
    prev_in_bore = False
    prev_angle = "start_vector_body_in_FOV" # The presence of this flag in end_angles indicates that the body's first position is already within the FOV cone considered 
    end_angles = []
    boresight_vector = np.array([1,0,0])
    fixed_frame_boresight_vector = quat.transform_vect_coord_system(boresight_vector, quat.quat_conjugate(q))

    # Other body positions are set to have default position vector [body_radius * 10, 0, 0] in origin frame:
    sun_vec = [RAD_SUN * 10, 0, 0]
    earth_vec = [RAD_EARTH * 10, 0, 0]
    moon_vec = [RAD_MOON * 10, 0, 0]

    if body_name == "sun":
        dict_ref = "sun_in_exclusion"
    elif body_name == "earth":
        dict_ref = "earth_in_fov"
    elif body_name == "moon":
        dict_ref = "moon_in_fov"

    while num_of_inc <= step_num:
        body_vec = body_start + num_of_inc * body_inc
        if body_name == "sun":
            sun_vec = body_vec
        elif body_name == "earth":
            earth_vec = body_vec
        elif body_name == "moon":
            moon_vec = body_vec
        else:
            raise ValueError("body_name must be one of 'sun', 'earth', or 'moon'.")

        res = str.measure(q, body_rates, sun_vec, earth_vec, moon_vec)
        body_angle = gc.angle_between_vectors(fixed_frame_boresight_vector, body_vec)

        if res[dict_ref] == True and prev_in_bore == False:
            end_angles.append(prev_angle)
        elif res[dict_ref] == False and prev_in_bore == True:
            end_angles.append(body_angle)
        
        prev_in_bore = res[dict_ref]
        prev_angle = body_angle
        num_of_inc += 1 

    return end_angles

def get_combinations(combo_digits: list, combo_len: int) -> list:
    """
    Return a list containing all combinations of combo_digits with length combo_len.
    Includes combinations where digits in combo_digits are repeated.
    """
    L = []
    if combo_len == 1:
        for digit in combo_digits:
            L.append([digit])
    else:
        base_combos = get_combinations(combo_digits, combo_len-1)
        for combo in base_combos:
            for digit in combo_digits:
                L.append(combo + [digit])
    return L

def test_sun_in_exclusion():
    """
    Verify whether the "sun_in_exclusion" parameter of STR measurements evaluates to True 
    in the appropriate conditions as the sun moves along several straight predefined paths.
    """
    str = VirtualSTR(STR_CFG)

    q = np.array([0,1,0,0])
    sun_vec_start = np.array([RAD_SUN * -2, 0, RAD_SUN * 10])
    sun_vec_end = np.array([RAD_SUN * -2, 0, RAD_SUN * -10])
    end_angles = get_cone_edge_angles(str, q, sun_vec_start, sun_vec_end, "sun", 10000)

    assert len(end_angles) == 2
    assert end_angles[0] == pytest.approx(0.980323067373, abs=0.001) # These values were indepentently verified via separate calculations
    assert end_angles[1] == pytest.approx(0.980323067373, abs=0.001)

    sun_vec_start = np.array([RAD_SUN * -10, RAD_SUN * 10, RAD_SUN * 100])
    sun_vec_end = np.array([RAD_SUN * -10, RAD_SUN * 10, RAD_SUN * -100])
    end_angles = get_cone_edge_angles(str, q, sun_vec_start, sun_vec_end, "sun", 10000)

    assert len(end_angles) == 0

    # More tests can be added if desired

def test_earth_in_fov():
    """
    Verify whether the "earth_in_fov" parameter of STR measurements evaluates to True 
    in the appropriate conditions as the earth moves along several straight predefined paths.
    """
    str = VirtualSTR(STR_CFG)

    q = np.array([0,1,0,0])
    earth_vec_start = np.array([RAD_EARTH * -10, RAD_EARTH * 10, RAD_EARTH * 10])
    earth_vec_end = np.array([RAD_EARTH * -10, RAD_EARTH * -10, RAD_EARTH * -10])
    end_angles = get_cone_edge_angles(str, q, earth_vec_start, earth_vec_end, "earth", 10000)

    assert len(end_angles) == 2
    assert end_angles[0] == pytest.approx(0.316852609851, abs=0.001)
    assert end_angles[1] == pytest.approx(0.316852609851, abs=0.001)

    earth_vec_start = np.array([RAD_EARTH * -10, RAD_EARTH * 3.5, RAD_EARTH * 100])
    earth_vec_end = np.array([RAD_EARTH * -10, RAD_EARTH * 3.5, RAD_EARTH * -100])
    end_angles = get_cone_edge_angles(str, q, earth_vec_start, earth_vec_end, "earth", 10000)

    assert len(end_angles) == 0

    # More tests can be added if desired

def test_moon_in_fov():
    """
    Verify whether the "moon_in_fov" parameter of STR measurements evaluates to True 
    in the appropriate conditions as the moon moves along several straight predefined paths.
    """
    str = VirtualSTR(STR_CFG)

    q = np.array([0,1,0,0])
    moon_vec_start = np.array([RAD_EARTH * -10, RAD_EARTH * 10, RAD_EARTH * 10])
    moon_vec_end = np.array([RAD_EARTH * -10, RAD_EARTH * -10, RAD_EARTH * -10])
    end_angles = get_cone_edge_angles(str, q, moon_vec_start, moon_vec_end, "moon", 10000)

    assert len(end_angles) == 2
    assert end_angles[0] == pytest.approx(0.248082125761, abs=0.001)
    assert end_angles[1] == pytest.approx(0.248082125761, abs=0.001)

    moon_vec_start = np.array([RAD_EARTH * -10, RAD_EARTH * 3, RAD_EARTH * 100])
    moon_vec_end = np.array([RAD_EARTH * -10, RAD_EARTH * 3, RAD_EARTH * -100])
    end_angles = get_cone_edge_angles(str, q, moon_vec_start, moon_vec_end, "moon", 10000)

    assert len(end_angles) == 0

    # More tests can be added if desired

def test_STR_rate_exceeded():
    """
    Verify that the STR class returns appropriate data at various body_rate values.
    """
    str = VirtualSTR(STR_CFG)

    q = np.array([0,1,0,0])
    sun_vec = np.array([0, 0, RAD_SUN * 10])
    earth_vec = np.array([0, 0, RAD_EARTH * 10])
    moon_vec = np.array([0, 0, RAD_MOON * 10])
    body_rates = np.array([-0.1, 0.03085335373721, 0])
    
    while body_rates[0] < 0.1:
        res = str.measure(q, body_rates, sun_vec, earth_vec, moon_vec)
        if body_rates[0] > 0.03085335373721:
            assert res["rate_exceeded"] == True
            assert np.array_equal(res["q_meas"], np.array([0,0,0,1]))
        elif body_rates[0] > -0.03085335373721:
            assert res["rate_exceeded"] == False
        else:
            assert res["rate_exceeded"] == True
            assert np.array_equal(res["q_meas"], np.array([0,0,0,1]))

        body_rates[0] += 0.0001

def test_edge_cases():
    """
    Verify that the STR class returns unit quaternions for a wide variety of input true quaternion values.
    """
    str = VirtualSTR(STR_CFG)

    sun_vect = np.array([RAD_SUN*10, RAD_SUN*10, RAD_SUN*10])
    earth_vect = np.array([RAD_EARTH*(-10), RAD_EARTH*10, RAD_EARTH*10])
    moon_vect = np.array([RAD_MOON*10, RAD_MOON*10, RAD_MOON*(-10)])
    body_rates = np.array([0,0,0])

    test_values = [-1, -(10**(-10)), 0, 0.5, 1, 10**10]
    testing_quats = get_combinations(test_values, 4)

    for q in testing_quats:
        q = np.asarray(q)
        if np.linalg.norm(q) == 0.0:
            continue

        # Testing non-unit quaternion inputs, as they should be automatically normalized by str.measure():
        dict = str.measure(q, body_rates, sun_vect, earth_vect, moon_vect) 

        q_meas = dict["q_meas"]
        assert len(q_meas) == 4
        assert np.linalg.norm(q_meas) == pytest.approx(1.0)

def test_noise_consistency():
    """
    Verify that the STR class returns quaternions with statistically consistent noise.
    """
    str = VirtualSTR(STR_CFG)

    sun_vect = np.array([RAD_SUN*10, RAD_SUN*10, RAD_SUN*10])
    earth_vect = np.array([RAD_EARTH*(-10), RAD_EARTH*10, RAD_EARTH*10])
    moon_vect = np.array([RAD_MOON*10, RAD_MOON*10, RAD_MOON*(-10)])
    body_rates = np.array([0,0,0])

    along_bore_std = math.sqrt(str.cov_rad2[0][0])

    cases = 0
    within_1_std = 0
    within_2_std = 0
    within_3_std = 0

    q_test = np.array([1,1,1,1])
    q_test = quat.normalize_quat(q_test)

    for i in range(10000):             
        dict = str.measure(q_test, body_rates, sun_vect, earth_vect, moon_vect)
        
        invalid_measurement = (
            dict["rate_exceeded"]
            or dict["sun_in_exclusion"]
            or dict["earth_in_fov"]
            or dict["moon_in_fov"]
        )
        
        if not invalid_measurement:
            q_meas = dict["q_meas"]
            cases += 1
        else:
            continue

        q_error = quat.quat_multiply(quat.quat_conjugate(q_test), q_meas)

        rot_vec_error = quat.rotvec_from_quat(q_error)

        if abs(rot_vec_error[0]) < along_bore_std:
            within_1_std += 1
        if abs(rot_vec_error[0]) < along_bore_std * 2:
            within_2_std += 1
        if abs(rot_vec_error[0]) < along_bore_std * 3:
            within_3_std += 1

    # The following assertions should only raise errors very infrequently:
    assert within_1_std / cases == pytest.approx(0.68, abs=0.02)
    assert within_2_std / cases == pytest.approx(0.95, abs=0.01)
    assert within_3_std / cases == pytest.approx(0.997, abs=0.005)

def test_str_digitization_precision():
    """
    Verify that STR measurements adhere to the sensor's digitization precision.
    """
    # TODO: Once quantization logic is added to sensors.py, ensure that returned 
    # noisy quaternions are of the appropriate precision (see also FSS unit tests).
    pytest.skip("Skipping until quantization step size details are finalized.")
