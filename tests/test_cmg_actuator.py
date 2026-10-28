#cmgtest
import pytest
import numpy as np
from pathlib import Path
from hardware.actuators.actuators import VirtualCMG


@pytest.fixture
def cmg():
    return VirtualCMG(Path("dummy_config.json"))


def test_cmg_initialization(cmg):
    #checking whether the  CMG identifies its model correctly
    assert cmg.model == "Generic CMG"


def test_cmg_cfg_path(cmg):
    #verifying the config path
    assert cmg.cfg_file == Path("dummy_config.json")


def test_cmg_open_config(cmg):
    #verifying the behavior of config load.
    with pytest.raises(NotImplementedError):
        cmg._load_cfg(Path("dummy_config.json"))


def test_cmg_zero_input(cmg):
    #checking the response to a zero command.
    result = cmg.command(np.zeros(3))

    # main code is not implemented yet, so this has been put for now
    assert result is None


def test_cmg_large_input(cmg):
    #checking the current response to very large inputs.
    command_input = np.array([1e100, -1e100, 1e100])

    result = cmg.command(command_input)

    # main code is not implemented yet, so this has been put for now
    assert result is None


@pytest.mark.parametrize(
    "command_input",
    [
        np.array([]),
        np.array([1.0]),
        np.array([1.0, 2.0]),
        np.array([1.0, 2.0, 3.0, 4.0]),
        np.zeros((3, 3)),
    ],
    ids=[
        "empty",
        "one_element",
        "two_elements",
        "four_elements",
        "two_dimensional",
    ],
)
def test_cmg_extreme_input_shapes(cmg, command_input):
    #checking behaviour for unusual input shapes
    result = cmg.command(command_input)

    # main code is not implemented yet, so this has been put for now
    assert result is None


@pytest.mark.parametrize(
    "command_input",
    [
        np.array([np.nan, 0.0, 0.0]),
        np.array([np.inf, 0.0, 0.0]),
        np.array([-np.inf, 0.0, 0.0]),
    ],
    ids=["nan", "positive_infinity", "negative_infinity"],
)
def test_cmg_nonfinite_inputs(cmg, command_input):
    #checking current behavior for NaN and infinite inputs
    result = cmg.command(command_input)
    assert result is None


def test_cmg_repeated_commands(cmg):
    #checking repeatability. Like if repeated inputs are given sequentially, the output
    #should be the same
    command_input = np.array([0.1, -0.2, 0.3])

    result1 = cmg.command(command_input)
    result2 = cmg.command(command_input)

    # main code is not implemented yet, so this has been put for now
    assert result1 is None
    assert result2 is None


def test_cmg_not_modify_input(cmg):
    #making a copy of the orginal input, then checking whether the processed 
    #input is same as the copy(unprocessed)
    command_input = np.array([0.1, -0.2, 0.3])
    original = command_input.copy()

    cmg.command(command_input)

    np.testing.assert_array_equal(command_input, original)
