""" Test the interface time of VREP for getting feedback
and sending in a control signal. """

import numpy as np
import time

import abr_control

robot_config = abr_control.arms.ur5.config()
# create our VREP interface
interface = abr_control.interfaces.vrep.interface(
    robot_config=robot_config, dt=0.001)
interface.connect()

num_timesteps = 100
times = np.zeros(num_timesteps)
try:
    for ii in range(num_timesteps):
        start_t = time.time()
        # get feedback
        feedback = interface.get_feedback()
        # send in control signal
        interface.apply_u(np.random.random(robot_config.num_joints))

        times[ii] = time.time() - start_t

finally:
    interface.disconnect()

print('Average interface time: %.5f' % (np.sum(times) / num_timesteps))
