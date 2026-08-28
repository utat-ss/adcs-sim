import pytest
import numpy as np
from root_finding import newton_method

FUNC_MAP = {
    "f(x) = x^2 - 4": lambda x: x**2 - 4,
    "f'(x) = 2x": lambda x: 2 * x,
    
    "f(x) = x - 0.5 * sin(x) - 1.0": lambda x: x - 0.5 * np.sin(x) - 1.0,
    "f'(x) = 1.0 - 0.5 * cos(x)": lambda x: 1.0 - 0.5 * np.cos(x),
    
    "f(x) = exp(x) - 2": lambda x: np.exp(x) - 2,
    "f'(x) = exp(x)": lambda x: np.exp(x),
    
    "f(x) = x^2 + 4": lambda x: x**2 + 4,
    
    "f(x) = x^3 - 2x + 2": lambda x: x**3 - 2 * x + 2,
    "f'(x) = 3x^2 - 2": lambda x: 3 * (x**2) - 2
}

test_samples = [
    {
    # testing for a standard case for standard convergence
    "function_string": "f(x) = x^2 - 4",
    "derivative_string": "f'(x) = 2x",
    "initial_guess": 3.0,
    "expected_root": 2.0,
    "expected_status": "converged"
    },
    {
    # testing for a standard case for transcendental functions
    "function_string": "f(x) = x - 0.5 * sin(x) - 1.0",
    "derivative_string": "f'(x) = 1.0 - 0.5 * cos(x)",
    "initial_guess": 1.0,
    "expected_root": 1.498701,
    "expected_status": "converged"
    },
    {
    # testing for exact guesses
    "function_string": "f(x) = x^2 - 4",
    "derivative_string": "f'(x) = 2x",
    "initial_guess": 2.0,
    "expected_root": 2.0,
    "expected_status": "converged"
    },
    {
    # testing for extremely far initial guesses
    "function_string": "f(x) = exp(x) - 2",
    "derivative_string": "f'(x) = exp(x)",
    "initial_guess": 10.0,
    "expected_root": 0.693147,
    "expected_status": "converged"
    },
    {
    # testing for d_func = 0
    "function_string": "f(x) = x^2 - 4",
    "derivative_string": "f'(x) = 2x",
    "initial_guess": 0.0,
    "expected_root": None,
    "expected_status": "raises_zero_division_error"
    },
    {
    # testing for no root
    "function_string": "f(x) = x^2 + 4",
    "derivative_string": "f'(x) = 2x",
    "initial_guess": 1.0,
    "expected_root": None,
    "expected_status": "does_not_converge"
    },
    {
    # testing for the infinite bounce case
    "function_string": "f(x) = x^3 - 2x + 2",
    "derivative_string": "f'(x) = 3x^2 - 2",
    "initial_guess": 0.0,
    "expected_root": None,
    "expected_status": "does_not_converge"
    },
    {
    # testing for zero tolerance
    "function_string": "f(x) = x^2 - 4",
    "derivative_string": "f'(x) = 2x",
    "initial_guess": 3.0,
    "tolerance": 0.0,
    "max_iter": 20,
    "expected_root": 2.0,
    "expected_status": "converged_via_max_iter"
    },
    {
    # testing for zero iterations
    "function_string": "f(x) = x^2 - 4",
    "derivative_string": "f'(x) = 2x",
    "initial_guess": 3.0,
    "tolerance": 1e-10,
    "max_iter": 0,
    "expected_root": 3.0,
    "expected_status": "converged_via_max_iter"
    },
    {
    # testing for one iteration
    "function_string": "f(x) = x^2 - 4",
    "derivative_string": "f'(x) = 2x",
    "initial_guess": 3.0,
    "tolerance": 1e-10,
    "max_iter": 1,
    "expected_root": 2.83333333,
    "expected_status": "converged_via_max_iter"
    },
]

@pytest.mark.parametrize("sample", test_samples)
def test_newton_method_cases(sample):
    func = FUNC_MAP[sample["function_string"]]
    d_func = FUNC_MAP[sample["derivative_string"]]
    
    x_n = sample["initial_guess"]
    expected_status = sample["expected_status"]
    expected_root = sample["expected_root"]

    if expected_status == "converged":
        result = newton_method(x_n, func, d_func)
        assert np.isclose(result, expected_root, atol=1e-5)

    elif expected_status == "converged_via_max_iter":
        tolerance = sample["tolerance"]
        max_iter = sample["max_iter"]
        result = newton_method(x_n, func, d_func, tolerance=tolerance, max_iter=max_iter)
        assert np.isclose(result, expected_root, atol=1e-5)

    elif expected_status == "raises_zero_division_error":
        with pytest.raises(ZeroDivisionError):
            newton_method(x_n, func, d_func)

    elif expected_status == "does_not_converge":
        result = newton_method(x_n, func, d_func)
        assert abs(func(result)) > 1e-3