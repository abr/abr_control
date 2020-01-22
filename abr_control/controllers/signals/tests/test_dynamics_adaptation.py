import pytest

import nengo
import numpy as np

from abr_control.arms import ur5
from abr_control.controllers import signals
from abr_control.controllers.signals import DynamicsAdaptation
from abr_control._vendor.nengolib.stats import ScatteredHypersphere


def test_scaling():
    robot_config = ur5.Config(use_cython=True)

    # looking at data centered around 50 with range of 25 each way
    means = np.ones(robot_config.N_JOINTS) * 50
    variances = np.ones(robot_config.N_JOINTS) * 25

    for ii in range(50):
        # test without spherical conversion
        adapt = DynamicsAdaptation(
            robot_config.N_JOINTS,
            robot_config.N_JOINTS,
            means=means,
            variances=variances,
            spherical=False,
        )

        unscaled = np.random.random(robot_config.N_JOINTS) * 50 + 25
        scaled = adapt.scale_inputs(unscaled)
        assert np.all(-1 < scaled) and np.all(scaled < 1)

        # test spherical conversion
        adapt = DynamicsAdaptation(
            robot_config.N_JOINTS,
            robot_config.N_JOINTS,
            means=means,
            variances=variances,
            spherical=True,
        )

        unscaled = np.random.random(robot_config.N_JOINTS) * 50 + 25
        scaled = adapt.scale_inputs(unscaled)
        assert np.linalg.norm(scaled) - 1 < 1e-5
