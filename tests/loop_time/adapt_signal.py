""" Test the runtime of the dynamics adaptation signal, for a given
number of neurons and Nengo backend. """

import numpy as np
import time

import abr_control

# ----TEST PARAMETERS-----
name = 'adapt_test'
num_trials = 1000
backend = 'nengo_ocl'  # can be nengo, nengo_ocl, nengo_spinnaker
regen = False
# ------------------------

# initialize our robot config
robot_config = abr_control.arms.jaco2.config_neural(
    regenerate_functions=regen)

# instantiate the adaptive controller
adapt = abr_control.controllers.signals.dynamics_adaptation(
    robot_config, n_neurons=1000, backend=backend)

# run once to generate the functions we need
adapt.generate(
    q=np.zeros(6), dq=np.zeros(6),
    training_signal=np.zeros(6))

times = np.zeros(num_trials)
for ii in range(num_trials):
    if (ii % 100) == 0:
        print('Iteration ', ii)

    # generate control signal
    start_time = time.time()
    u = adapt.generate(
        q=np.random.random(robot_config.num_joints),
        dq=np.random.random(robot_config.num_joints),
        training_signal=np.random.random(6))
    times[ii] = time.time() - start_time

avg_time = np.sum(times) / num_trials
print('Average control loop time: %.6f' % avg_time)
