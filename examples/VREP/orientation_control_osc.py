"""
Running operational space control using VREP. The controller will
move the end-effector to the target object's orientation.
"""
import numpy as np
import pygame

from abr_control.arms import ur5 as arm
from abr_control.controllers import OSC
from abr_control.interfaces import VREP
from abr_control.utils import transformations


# initialize our robot config
robot_config = arm.Config(use_cython=True)
# create opreational space controller
ctrlr = OSC(robot_config, kp=50)

# create our interface
interface = VREP(robot_config, dt=.005)
interface.connect()

# set up lists for tracking data
ee_track = []

# control (alpha, beta, gamma) out of [x, y, z, alpha, beta, gamma]
ctrlr_dof = [False, False, False, True, True, True]


try:
    count = 0
    print('\nSimulation starting...\n')
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        target = np.hstack([
            interface.get_xyz('target'),
            interface.get_orientation('target')])

        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            ctrlr_dof=ctrlr_dof,
            )

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # track data
        ee_track.append(np.copy(hand_xyz))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')

    ee_track = np.array(ee_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from abr_control.utils.plotting import plot_3D

        plot_3D(ee_track)
        plt.show()
