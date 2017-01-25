"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1.5 simulated seconds, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np
import signal
import sys

import abr_control

# initialize our robot config for the ur5
robot_config = abr_control.arms.ur5.config(regenerate_functions=True)
# instantiate controller
ctrlr = abr_control.controllers.osc(robot_config, kp=1000, vmax=None)
# create our VREP interface
interface = abr_control.interfaces.vrep(robot_config, dt=.001)
interface.connect()

# set up lists for tracking data
ee_track = []
target_track = []

def on_exit(signal, frame):
    """ A function for plotting the end-effector trajectory and error """
    global ee_track, target_track
    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # plot start point of hand
    ax.plot(*ee_track[0], 'bx', mew=10)
    # plot trajectory of hand
    ax.plot(*ee_track)
    # plot trajectory of target
    ax.plot(*target_track, 'rx', mew=10)
    ax.set_xlim3d(-1, 1)
    ax.set_ylim3d(-1, 1)
    ax.set_zlim3d(0, 1.5)

    plt.figure()
    plt.plot(np.sqrt(np.sum((np.array(target_track) -
                             np.array(ee_track))**2, axis=1)))
    plt.ylabel('Error (m)')
    plt.xlabel('Time (ms)')
    plt.show()
    sys.exit()

# call on_exit when ctrl-c is pressed
signal.signal(signal.SIGINT, on_exit)

try:
    num_targets = 0
    back_to_start = False

    # create a target based off initial arm position
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', q=feedback['q'])
    target_xyz = start + np.array([.25, -.25, 0.0])
    interface.set_xyz(name='target', xyz=target_xyz)
    # get the target orientation to match
    target_angles = interface.get_orientation(name='target')
    target_quat = abr_control.utils.transformations.quaternion_from_euler(
        target_angles[0], target_angles[1], target_angles[2], 'rxyz')

    count = 0.0
    while 1:
        # get arm feedback from VREP
        feedback = interface.get_feedback()

        # generate control signal
        u = ctrlr.control(
            q=feedback['q'],
            dq=feedback['dq'],
            target_x=target_xyz,
            target_quat=target_quat,
            mask=[0, 0, 0, 1, 1, 1])

        # apply the control signal, step the sim forward
        interface.apply_u(u)

        ee_xyz = interface.get_xyz('hand')
        angles = interface.get_orientation('UR5_link6')
        quat_from_vrep = abr_control.utils.transformations.quaternion_from_euler(
            angles[0], angles[1], angles[2], 'rxyz')
        print('quat_from_vrep: ', quat_from_vrep)
        print('quat from config: ', robot_config.orientation('EE', feedback['q']))
        print('target quat: ', target_quat)
        # print('orientation: ', angles)
        # print('target orientation: ', target_angles)
        interface.set_orientation('hand', angles)
        print('error: ', np.sqrt(np.sum((target_xyz - ee_xyz)**2)))
        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target_xyz))

        count += 1

finally:
    # stop and reset the VREP simulation
    interface.disconnect()

on_exit()
