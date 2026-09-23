from collections.abc import Callable


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