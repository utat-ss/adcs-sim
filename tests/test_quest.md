Ok.
The reason I'm creating this file is because my dumaas can't keep track of shit in my working memory
So...WTF am I doing?
According to the issue:

Develop unit tests for the QUEST state estimation approach to determine whether it functions properly under a wide range of conditions, including edge cases.

Use pytest and place them in a tests folder at the top level of the project.
Things to test for:

Outputs are mathematically consistent with expectations
Outputs are kinematically correct (e.g. quaternions are unitary)
Invalid inputs are handled gracefully

Well...this is a tests folder...so I develop the pytest and place them in test_quest.py i think.

The files I'm concerning myself with are State_estimation/QUEST_algorithm.py
and utils/quaternion_math/ function normalize_quat

First of let's look at the math section, since I can actually judge that

``` python

def normalize_quat(q: np.ndarray) -> np.ndarray:
    """
    Normalize quaternion q = [x, y, z, w].
    """
    q = np.asarray(q, dtype=float)

    if q.shape != (4,):
        raise ValueError(f"Quaternion must have shape (4,), got {q.shape}")

    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError("Quaternion has zero norm.")

    return q / norm

```

Let's mathematize it...

$$
q = [x,y,z,w]
$$
each value in q is a float.

linalg.norm is quite simple since it just calculates the L-2 norm of this system.

As per "Invalid inputs are handled gracefully", I first have to check what if q is not an array. But, maybe not because it is a numerical helper function.
I do however need to check what happens if the norm is exceptionally close to 0.
Say 1e-12, does it blow up. Or is it managed.
(Just a note, it isn't but why not...we'll just send it for testing)

``` python
import numpy as np
from utils.quaternion_math import normalize_quat


def QUEST(V: np.ndarray, W: np.ndarray):
    """
    Estimate spacecraft attitude using Shuster's QUEST algorithm.

    Each column of V and W represents one observation vector.
    The i-th column of V and the i-th column of W must correspond
    to the same physical observation.

    :param V: np.ndarray
        Reference observation vectors expressed in the chosen
        reference frame. Expected shape is (3, m), where m is
        the number of observations.
    :param W: np.ndarray
        Corresponding measured observation vectors expressed in
        the spacecraft body frame. Expected shape is (3, m).
    :return: Quaternions indicating current attitude
    """
    if V.shape != W.shape:
        raise ValueError("In correct length, V and W should have same number of element")
    B = W @ V.T
    sigma = np.trace(B)
    H = B + B.T
    z = np.array([
        B[1, 2] - B[2, 1],
        B[2, 0] - B[0, 2],
        B[0, 1] - B[1, 0],
    ])
    kappa = 0.5 * (np.trace(H) ** 2 - np.trace(H @ H))
    a = sigma ** 2 - kappa
    b = sigma ** 2 + z.T @ z
    c = np.linalg.det(H) + z.T @ H @ z
    d = z.T @ H @ H @ z
    eigval = np.roots([1, 0, -a-b, -c, a * b + c * sigma - d]).real
    eigval_max = np.max(eigval)
    M = (eigval_max + sigma) * np.identity(3) - H
    p = np.linalg.solve(M, z)
    q = np.concatenate(([1.0], p))
    q = normalize_quat(q)
    return q
```

Here's what this quest algorithm does

Step 1: Takes 2 3 dimensional vectors (V and W)
Step 2: Take the dot product of the vectors V and W
Step 3: Matrix multiply V and W^T
Step 4: Sum their elements along their diagnoals to get a scalar per each vector
Step 5: Constructs a symmetric matrix
Step 6: Extracts anti-symmetric (skew-symmetric) components of B
Step 7:  leverages Faddeev-LeVerrier's algorithm to compute the trace of the adjugate matrix (I don't know the algorithm but that is what Google says)
Step 8: Takes the trace of the original matrix and subtracts the new trace 
Step 9: Adds the trace of B to the skew symmetric norm
Step 10: 

Fuck it, we'll just refer to the NASA document
https://ntrs.nasa.gov/api/citations/19960035750/downloads/19960035750.pdf
shit, my bad...that's REQUEST

QUEST is just this

https://malcolmdshuster.com/Pub_2006f_J_Brouwer_AAS.pdf

Ok...let's just get in some test cases then...
First off

QUEST was used in the MagSat missions...So I might as well rip some parameters from there.

Also..just a note the current implementation of normalize_quat does not handle values near zero.
So...erronious sensor data may be interpreted as non erronious and we'll waste computation...

The reference I'm going to be referring to is the same one I think the original author referred to
but I might be wrong

Three-Axis Attitude Determination from Vector Observations,” M. D. Shuster and S. D.
Oh, Journal of Guidance and Control, Vol. 4, No. 1, January–February 1981, pp. 70–77.

Ok...and we have a few things pop up that I might want to test for

If the true spacecraft rotation happens to approach a 180 degree rotation, the scalar component q4 approaches zero.
Loss of precision when doing np.roots
Collinear Input vectors

EVERY SENSOR IS TREATED AS EQUALLY RELIABLE. (Which to be fair...I wouldn't know how to categorize)

Insufficient Observation Tests
High Noise and Ill conditioned tests.
