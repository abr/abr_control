""" Tests the speed of the osc controller running on the Jaco2. """

import numpy as np
import time

import abr_control

# initialize our robot config for the ur5
robot_config = abr_control.arms.jaco2.config(
    regenerate_functions=True, cython=True)

# create our VREP interface
interface = abr_control.interfaces.vrep(robot_config, dt=.001)

# instantiate the REACH controller with obstacle avoidance
ctrlr = abr_control.controllers.osc(robot_config, kp=200, vmax=0.5)
# generate the functions we need
ctrlr.control(q=np.zeros(6), dq=np.zeros(6), target_state=np.ones(6))

try:
    interface.connect()
    # create a target based on initial arm position
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', q=feedback['q'])
    target_xyz = start + np.array([-.25, .25, 0])
    interface.set_xyz(name='target', xyz=target_xyz)

    num_timesteps = 1500
    times = np.zeros(num_timesteps)
    for ii in range(num_timesteps):
        start_time = time.time()
        # get arm feedback from VREP
        feedback = interface.get_feedback()

        # use visual feedback to get object endpoint position
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])

        # generate control signal
        u = ctrlr.control(
            q=feedback['q'],
            dq=feedback['dq'],
            target_x=target_xyz,
            target_dx=np.zeros(3))

        # apply the control signal, step the sim forward
        interface.apply_u(u)
        times[ii] = times.time() - start_time
finally:
    # stop and reset the VREP simulation
    interface.disconnect()

    print('Average loop time: %.5f' % (np.sum(times) / num_timesteps))
