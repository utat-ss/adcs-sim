import math
import numpy as np

def angle_between_vectors(vector_1: np.ndarray, vector_2: np.ndarray) -> float:
    """
    Compute the angle [rad] between two nonzero vectors of the same dimension.
    """
    dot_p = np.dot(vector_1, vector_2)
    norm_1 = np.linalg.norm(vector_1)
    norm_2 = np.linalg.norm(vector_2)
    if norm_1 == 0.0 or norm_2 == 0.0:
        raise ValueError("Both vector_1 and vector_2 must have nonzero magnitude.")

    cos_theta = dot_p / (norm_1 * norm_2)
    if cos_theta > 1.0 or cos_theta < -1.0:
        cos_theta = round(cos_theta, 15)
    vector_angle_rad = math.acos(cos_theta)

    return vector_angle_rad

def project_vector(vector_1: np.ndarray, vector_2: np.ndarray) -> np.ndarray:
    """
    Compute the vector projection of vector_1 onto vector_2, where vector_2 is nonzero.
    """

    if np.linalg.norm(vector_2) == 0.0:
        raise ValueError("vector_2 must have nonzero magnitude.")

    return (np.dot(vector_1, vector_2) / np.dot(vector_2, vector_2)) * vector_2

def determine_circle_tangent_vectors(circle_center_vector: np.ndarray, circle_radius: float) -> tuple:
    """
    Compute the two 2D position vectors (starting from the origin) whose endpoints are tangent 
    to a given circle of radius circle_radius centered at circle_center_vector.
    """

    normed_vec = np.linalg.norm(circle_center_vector)
    tangent_dot_body = normed_vec ** 2 - (circle_radius ** 2)
    if tangent_dot_body <= 0.0:
        raise ValueError("planar_body_vector must be of a greater magnitude than body_radius.")
    
    temp = math.sqrt(tangent_dot_body) * circle_radius
    
    tangent_1_coord_1 = (tangent_dot_body * circle_center_vector[0] + circle_center_vector[1] * temp) / (normed_vec ** 2)
    tangent_1_coord_2 = (tangent_dot_body * circle_center_vector[1] - circle_center_vector[0] * temp) / (normed_vec ** 2)

    tangent_2_coord_1 = (tangent_dot_body * circle_center_vector[0] - circle_center_vector[1] * temp) / (normed_vec ** 2)
    tangent_2_coord_2 = (tangent_dot_body * circle_center_vector[1] + circle_center_vector[0] * temp) / (normed_vec ** 2)

    return (np.array([tangent_1_coord_1, tangent_1_coord_2]), np.array([tangent_2_coord_1, tangent_2_coord_2]))

def to_planar_vector(sat_vector: np.ndarray, body_vector: np.ndarray) -> np.ndarray:
    """
    Compute the 2D position vector representing body_vector in a coordinate system 
    within the origin-intersecting plane defined by 3D vectors sat_vector and body_vector. 
    The basis for this planar coordinate system is the set of unit vectors {sat_vector / |sat_vector|, 
    (sat_vector x (body_vector x sat_vector)) / |sat_vector x (body_vector x sat_vector)|}.
    """
    
    coord_1 = np.dot(sat_vector, body_vector) / np.linalg.norm(sat_vector)
    temp = (np.linalg.norm(body_vector) ** 2) - (coord_1 ** 2)
    if temp < 0.0:
        temp = round(temp, 10)
        if temp < 0.0:
            raise ValueError("Invalid vectors entered into function.")
        
    coord_2 = math.sqrt(temp)

    return np.array([coord_1, coord_2])

def from_planar_vector(defining_vector_1_3D: np.ndarray, defining_vector_2_3D: np.ndarray, in_plane_vector_2D: np.ndarray) -> np.ndarray:
    """
    Compute the 3D position vector representing the 2D vector in_plane_vector_2D within the 
    origin-intersecting plane defined by 3D vectors defining_vector_1_3D and defining_vector_2_3D. 
    The basis for the original planar coordinate system used is the set of unit vectors {defining_vector_1_3D / |defining_vector_1_3D|, 
    (defining_vector_1_3D x (defining_vector_2_3D x defining_vector_1_3D)) / 
    |defining_vector_1_3D x (defining_vector_2_3D x defining_vector_1_3D)|}.
    """
    
    plane_normal = np.cross(defining_vector_2_3D, defining_vector_1_3D)
    dir_vect_2 = np.cross(defining_vector_1_3D, plane_normal)

    unit_vect_1 = defining_vector_1_3D / np.linalg.norm(defining_vector_1_3D)
    if np.array_equal(dir_vect_2, np.zeros(3)):
        unit_vect_2 = np.zeros(3)
    else:
        unit_vect_2 = dir_vect_2 / np.linalg.norm(dir_vect_2)

    return unit_vect_1 * in_plane_vector_2D[0] + unit_vect_2 * in_plane_vector_2D[1]

def planar_projection(vect: np.ndarray, plane_normal: np.ndarray) -> np.ndarray:
    """
    Compute the planar projection of 3D vector vect onto the plane defined by plane_normal.
    """
    
    plane_deviation = project_vector(vect, plane_normal)
    projection = vect - plane_deviation
    return projection

def sphere_plane_intersection(sphere_pos_vect: np.ndarray, sphere_radius: float, plane_normal: np.ndarray) -> tuple:
    """
    Compute a tuple containing the 3D position vector of a circle
    representing the intersection between an origin-intersecting plane and a sphere, 
    followed by said circle's radius. The sphere is centered at sphere_pos_vect, and has
    radius sphere_radius, while the plane has the normal vector plane_normal.

    Note: The returned circle has radius zero if no intersection occurs.
    """

    plane_deviation = project_vector(sphere_pos_vect, plane_normal)
    proj_circle_pos_vect = sphere_pos_vect - plane_deviation

    temp = sphere_radius ** 2 - np.linalg.norm(plane_deviation) ** 2
    if temp >= 0.0:
        proj_circle_radius = math.sqrt(temp)
    else:
        proj_circle_radius = 0.0

    return (proj_circle_pos_vect, proj_circle_radius)

def eval_vector_between_vectors(checked_vect: np.ndarray, boundary_vect_1: np.ndarray, boundary_vect_2: np.ndarray) -> bool:
    """
    Determine whether a given position vector checked_vect is within the cone 
    defined by position vectors boundary_vect_1 and boundary_vect_2.
    """
    angle_between_boundaries = angle_between_vectors(boundary_vect_1, boundary_vect_2)
    bound_to_checked_angle_1 = angle_between_vectors(checked_vect, boundary_vect_1)
    bound_to_checked_angle_2 = angle_between_vectors(checked_vect, boundary_vect_2)

    if round(angle_between_boundaries, 12) == round(bound_to_checked_angle_1 + bound_to_checked_angle_2, 12):
        return True
    return False

    # print(round(angle_between_boundaries, 12), round(bound_to_checked_angle_1 + bound_to_checked_angle_2, 12))







        # print("\n", vector_1, vector_2, vector_angle_rad/math.pi*180,"\n")