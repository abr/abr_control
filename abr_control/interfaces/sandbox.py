import numpy as np
import sys
import time

import abr_control

robot_config = abr_control.arms.ur5.config.robot_config(
    regenerate_functions=False, use_ufuncify=True)
Mq = robot_config.Mq(q=np.ones(6))

num_trials = 1000
times = np.empty(num_trials)
for ii in range(num_trials):
    start = time.time()
    Mq = robot_config.Mq(q=np.ones(6))
    times[ii] = time.time() - start

print('Average time: %.5f' % (np.sum(times) / num_trials))
