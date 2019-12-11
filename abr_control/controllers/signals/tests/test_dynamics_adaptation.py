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

@pytest.mark.parametrize('spherical', ((True), (False)))
@pytest.mark.parametrize('n_neurons', ((1000), (100)))
@pytest.mark.parametrize('n_ensembles', ((1), (4)))
@pytest.mark.parametrize('intercepts', (('manual'), (None)))
@pytest.mark.parametrize('encoders', (('manual'), (None)))
def test_intercepts_and_encoders(
        spherical, n_neurons, n_ensembles,
        intercepts, encoders):

    n_input = 2
    n_output = 3
    seed = 0

    if encoders == 'manual':
        # reset rng
        np.random.seed = seed

        hypersphere = ScatteredHypersphere(surface=True)
        encoders = hypersphere.sample(
            n_neurons * n_ensembles, n_input + spherical)

        encoders = encoders.reshape(
            n_ensembles, n_neurons, n_input + spherical)

    if intercepts == 'manual':
        # reset rng
        np.random.seed = seed

        triangular = np.random.triangular(
            left=0.3, mode=0.35, right=0.4, size=n_neurons*n_ensembles)
        intercepts = nengo.dists.CosineSimilarity(n_input + 2).ppf(1 - triangular)
        intercepts = intercepts.reshape((n_ensembles, n_neurons))

    adapt = DynamicsAdaptation(
        n_input=n_input,
        n_output=n_output,
        n_neurons=n_neurons,
        n_ensembles=n_ensembles,
        seed=seed,
        encoders=encoders,
        intercepts=intercepts,
        spherical=spherical)

    for ii in range(n_ensembles):
        enc_shape = adapt.sim.data[adapt.adapt_ens[ii]].encoders.shape
        assert np.array_equal(
            np.asarray(enc_shape), np.array([n_neurons, n_input + spherical]))

        int_shape = adapt.sim.data[adapt.adapt_ens[ii]].intercepts.shape
        assert np.array_equal(
            np.asarray(int_shape), np.array([n_neurons, ]))

    # check that our intercepts and encoders are the same to confirm the seed
    # has not changed
    adapt2 = DynamicsAdaptation(
        n_input=n_input,
        n_output=n_output,
        n_neurons=n_neurons,
        n_ensembles=n_ensembles,
        seed=seed,
        encoders=encoders,
        intercepts=intercepts,
        spherical=spherical)

    enc2_shape = adapt2.sim.data[adapt2.adapt_ens[ii]].encoders.shape
    int2_shape = adapt2.sim.data[adapt2.adapt_ens[ii]].intercepts.shape
    assert np.array_equal(enc2_shape, enc_shape)
    assert np.array_equal(int2_shape, int_shape)
