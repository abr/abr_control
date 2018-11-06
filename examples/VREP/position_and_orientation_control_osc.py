"""
Running operational space control using VREP. The controller will
move the end-effector to the target object's position and orientation.
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
ctrlr = OSC(robot_config, kp=200, vmax=10.0)

# create our interface
interface = VREP(robot_config, dt=.005)
interface.connect()

# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []

# control (x, y, beta, gamma) out of [x, y, z, alpha, beta, gamma]
ctrlr_dof = [True, True, False, False, True, True]


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
        ee_angles_track.append(transformations.euler_from_matrix(
            robot_config.R('EE', feedback['q'])))
        target_track.append(np.copy(target[:3]))
        target_angles_track.append(interface.get_orientation('target'))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')

    ee_track = np.array(ee_track)
    ee_angles_track = np.array(ee_angles_track)
    target_track = np.array(target_track)
    target_angles_track = np.array(target_angles_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from abr_control.utils.plotting import plot_3D

        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(ee_track)
        plt.gca().set_prop_cycle(None)
        plt.plot(target_track, '--')
        plt.ylabel('3D position (m)')

        plt.subplot(2, 1, 2)
        plt.plot(ee_angles_track)
        plt.gca().set_prop_cycle(None)
        plt.plot(target_angles_track, '--')
        plt.ylabel('3D orientation (rad)')
        plt.xlabel('Time (s)')

        plt.tight_layout()

        plot_3D(ee_track, target_track)
        plt.show()
