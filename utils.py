import math
import numpy as np

def angle_between_vectors(vector_1: np.ndarray, vector_2: np.ndarray) -> float:
    """to be added"""

    dot_p = np.dot(vector_1, vector_2)
    norm_1 = np.linalg.norm(vector_1)
    norm_2 = np.linalg.norm(vector_2)
    if norm_1 == 0.0 or norm_2 == 0.0:
        raise ValueError("Both vector_1 and vector_2 must have nonzero magnitude.")

    cos_theta = dot_p / (norm_1 * norm_2)
    vector_angle_rad = math.acos(cos_theta)
    return vector_angle_rad

def project_vector(vector_1: np.ndarray, vector_2: np.ndarray) -> np.ndarray:
    """Project vector_1 onto vector_2."""

    if np.linalg.norm(vector_2) == 0.0:
        raise ValueError("vector_2 must have nonzero magnitude.")

    return (np.dot(vector_1, vector_2) / np.dot(vector_2, vector_2)) * vector_2

def determine_body_tangent_vectors(planar_body_vector: np.ndarray, body_radius: float) -> tuple:
    """to be added"""

    normed_vec = np.linalg.norm(planar_body_vector)
    tangent_dot_body = normed_vec ** 2 - (body_radius ** 2)
    if tangent_dot_body <= 0.0:
        raise ValueError("planar_body_vector must be of a greater magnitude than body_radius.")
    
    temp = math.sqrt(tangent_dot_body) * body_radius
    
    tangent_1_coord_1 = (tangent_dot_body * planar_body_vector[0] + planar_body_vector[1] * temp) / (normed_vec ** 2)
    tangent_1_coord_2 = (tangent_dot_body * planar_body_vector[1] - planar_body_vector[0] * temp) / (normed_vec ** 2)

    tangent_2_coord_1 = (tangent_dot_body * planar_body_vector[0] - planar_body_vector[1] * temp) / (normed_vec ** 2)
    tangent_2_coord_2 = (tangent_dot_body * planar_body_vector[1] + planar_body_vector[0] * temp) / (normed_vec ** 2)

    return (np.array([tangent_1_coord_1, tangent_1_coord_2]), np.array([tangent_2_coord_1, tangent_2_coord_2]))

def to_planar_vector(sat_vector: np.ndarray, body_vector: np.ndarray) -> np.ndarray:
    """to be added"""
    
    coord_1 = np.dot(sat_vector, body_vector) / np.linalg.norm(sat_vector)
    temp = (np.linalg.norm(body_vector) ** 2) - (coord_1 ** 2)
    if temp < 0.0:
        temp = round(temp, 10)
        if temp < 0.0:
            raise ValueError("Invalid vectors entered into function.")
        
    coord_2 = math.sqrt(temp)

    return np.array([coord_1, coord_2])

def from_planar_vector(sat_vector: np.ndarray, body_vector_3D: np.ndarray, body_vector_2D: np.ndarray) -> np.ndarray:
    """to be added"""
    
    plane_normal = np.cross(body_vector_3D, sat_vector)
    dir_vect_2 = np.cross(sat_vector, plane_normal)

    unit_vect_1 = sat_vector / np.linalg.norm(sat_vector)
    if np.array_equal(dir_vect_2, np.zeros(3)):
        unit_vect_2 = np.zeros(3)
    else:
        unit_vect_2 = dir_vect_2 / np.linalg.norm(dir_vect_2)

    return unit_vect_1 * body_vector_2D[0] + unit_vect_2 * body_vector_2D[1]

def sphere_plane_intersection(sphere_pos_vect: np.ndarray, sphere_radius: float, plane_normal: np.ndarray):
    """to be added
    Plane passes through (0,0,0)
    """

    plane_deviation = project_vector(sphere_pos_vect, plane_normal)
    proj_circle_pos_vect = sphere_pos_vect - plane_deviation

    proj_circle_radius = math.sqrt(sphere_radius ** 2 - np.linalg.norm(plane_deviation) ** 2) # add error

    return (proj_circle_pos_vect, proj_circle_radius)
