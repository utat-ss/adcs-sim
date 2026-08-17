import numpy as np

"""
Input to the function is matrix A. The function determines LTI stability based on the eigenvalues
of the matrix.

If stable, it returns True and an empty list

If unstable, it returns False and a list with all the real parts of the eigenvalues
that are on the right half of the plane

"""

def lti_stability_analysis(matrix):
     eigenvalues = np.linalg.eigvals(matrix)
     if np.all(np.real(eigenvalues) < 0):
        return True, []
     
     else:
        unstable_values = []
        for i in eigenvalues:
            if np.real(i) >=0:
                unstable_values.append(np.real(i))
        return False, unstable_values
