"""For all functions, a short description is provided, with the code itself, below"""



"""

Python code for performing quaternion <-> Euler Angle conversions. 

Note that quaternion arguments must all be in scalar-last JPL convention, i.e., in the format of [x, y, z, w]. 
Euler angles must be expressed in the 3-2-1 Euler angle sequence [Yaw, Pitch, Roll]. 

Quaternions and euler angles are stored as 1D numpy arrays in the format of [x, y, z, w] and [Yaw, Pitch, Roll] respectively. 
For angles, radians are to be used. 
"""

import numpy as np 
from scipy.spatial.transform import Rotation as R

def eulerToQuat(euler): 

    rotationEndQuat = R.from_euler("ZYX", euler)

    quaternion = rotationEndQuat.as_quat()

    return quaternion


def quatToEuler(quaternion): 

    rotationEndEuler = R.from_quat(quaternion) 

    euler = rotationEndEuler.as_euler("ZYX")

    return euler 




"""

Python code for performing quaternion multiplication. 

Note that arguments must all be in scalar-last JPL convention, i.e., in the format of [x, y, z, w]. 

Quaternions are stored as 1D numpy arrays. 

"""


def quatMultiply(q1, q2): 


    #Unpack quaternion arrays into single variable components 
    x1, y1, z1, w1 = q1 
    x2, y2, z2, w2 = q2

    #Find new components for resultant quaternion
    h = w1*x2 + x1*w2 + y1*z2 - z1*y2
    j = w1*y2 - x1*z2 + y1*w2 + z1*x2
    k = w1*z2 + x1*y2 - y1*x2 + z1*w2
    g = w1*w2 - x1*x2 - y1*y2 - z1*z2

    q3 = np.array([h, j, k, g]) #Final resultant quaternion

    return q3



"""

Python code for performing quaternion normalization. 

Note that the arguments must all be in scalar-last JPL convention, i.e., in the format of [x, y, z, w]. 

Quaternions are stored as 1D numpy arrays. 

"""



def quatNormalize(q): 

    magnitude = np.linalg.norm(q) #Find Magnitude of Quaternion

    if magnitude == 0: #Handle division by 0

        qNorm = np.array([0, 0, 0, 1]) #Perform no transformation in the event of an attempted division by 0 

    else: 
        qNorm = q / magnitude #Compute norm

    return qNorm 




"""

Python code for performing quaternion <-> Rotation Matrix conversions. 

Note that quaternion arguments must all be in scalar-last JPL convention, i.e., in the format of [x, y, z, w]. 

Quaternions are stored as 1D numpy arrays in the format of [x, y, z, w]. 

Rotation matrix presented as numpy array. 

"""


def quatToRotationMatrix(quat): 

    magnitude = np.linalg.norm(quat)

    quat = quat / magnitude

    x, y, z, w = quat 

    R = np.array([[1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
                 [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
                 [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]])
    
    return R

def rotationMatrixToQuat(matrix): 

    rotation = R.from_matrix(matrix)

    quaternion = rotation.as_quat()

    return np.array(quaternion)