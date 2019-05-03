import pytest
import timeit

import matplotlib.pyplot as plt
import numpy as np

from abr_control.arms import twojoint, ur5, jaco2
from abr_control.controllers import OSC


@pytest.mark.parametrize('arm, config_params, osc_params', (
    (twojoint, {}, {}),
    (ur5, {}, {'ctrlr_dof':[True,]*6}),
    (jaco2, {'hand_attached':False}, {'ctrlr_dof':[True,]*5 + [False]}),
    (jaco2, {'hand_attached':True}, {'ctrlr_dof':[True,]*6})))
@pytest.mark.parametrize('use_cython', (False, True))
def test_timing(arm, config_params, osc_params, use_cython, plt):
    robot_config = arm.Config(use_cython=use_cython, **config_params)
    ctrlr = OSC(robot_config, **osc_params)

    # then run for timing
    n_trials = 100
    times = np.zeros(n_trials+1)
    for ii in range(n_trials+1):
        q = np.random.random(robot_config.N_JOINTS) * 2 * np.pi
        dq = np.random.random(robot_config.N_JOINTS) * 5
        target = np.random.random(6) * 2 - 1

        start = timeit.default_timer()
        ctrlr.generate(q=q, dq=dq, target=target)

        times[ii] = timeit.default_timer() - start

    # strip off first loop to remove load time
    times = times[1:]
    average_time = (np.sum(times) / n_trials)

    plt.plot(times)
    plt.title('Average time: %.5f' % average_time)
    plt.xlabel('Loop number')
    plt.ylabel('Time (s)')
