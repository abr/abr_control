import timeit

import matplotlib.pyplot as plt
import numpy as np

from abr_control.arms import twojoint, ur5, jaco2
from abr_control.controllers import OSC


def test_timing(arm, config_params, osc_params, use_cython):
    robot_config = arm.Config(use_cython=use_cython, **config_params)
    ctrlr = OSC(robot_config, **osc_params)

    # then run for timing
    n_trials = 1000
    times = np.zeros(n_trials + 1)
    for ii in range(n_trials + 1):
        q = np.random.random(robot_config.N_JOINTS) * 2 * np.pi
        dq = np.random.random(robot_config.N_JOINTS) * 5
        target = np.random.random(6) * 2 - 1

        start = timeit.default_timer()
        ctrlr.generate(q=q, dq=dq, target=target)

        times[ii] = timeit.default_timer() - start

    # strip off first loop to remove load time
    times = times[1:]
    average_time = np.sum(times) / n_trials

    return average_time


param_sets = (
    (twojoint, {}, {}),
    (ur5, {}, {"ctrlr_dof": [True,] * 6}),
    (jaco2, {"hand_attached": False}, {"ctrlr_dof": [True,] * 5 + [False]}),
    (jaco2, {"hand_attached": True}, {"ctrlr_dof": [True,] * 6}),
)

timing_tests_python = []
timing_tests_cython = []
logs = [timing_tests_python, timing_tests_cython]
for log, use_cython in zip(logs, [False, True]):
    for arm, config_params, osc_params in param_sets:
        log.append(test_timing(arm, config_params, osc_params, use_cython=use_cython))
# convert to milliseconds
timing_tests_python = np.array(timing_tests_python) * 1000
timing_tests_cython = np.array(timing_tests_cython) * 1000

n = len(timing_tests_python)
labels = ["Two joint", "UR5", "Jaco2", "Jaco2 with hand"]

plt.subplot(2, 1, 1)
python = plt.bar(np.arange(n), timing_tests_python, width=0.5)
plt.title("Timing test for generating OSC signal")
plt.ylabel("Time (ms)")
plt.legend([python], ["Python"], loc=2)
plt.xticks([])

plt.subplot(2, 1, 2)
cython = plt.bar(np.arange(n), timing_tests_cython, width=0.5, color="C1")
plt.ylabel("Time (ms)")
plt.legend([cython], ["Cython"], loc=2)
plt.xticks(np.arange(n), labels)

plt.tight_layout()
plt.show()
