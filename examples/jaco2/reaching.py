"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1.5 simulated seconds, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np
import signal
import sys

import abr_control

# initialize our robot config
robot_config = abr_control.arms.jaco2.config(
    regenerate_functions=True)

# instantiate controller
ctrlr = abr_control.controllers.osc(
    robot_config, kp=20, vmax=.5, null_control=False)

# create our VREP interface
interface = abr_control.interfaces.vrep(
    robot_config, dt=.001)
interface.connect()

# set up lists for tracking data
ee_track = []
target_track = []


def on_exit(signal, frame):
    """ A function for plotting the end-effector trajectory and error """
    global ee_track, target_track
    ee_track = np.array(ee_track)

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # plot start point of hand
    ax.plot([ee_track[0, 0]], [ee_track[0, 1]], [ee_track[0, 2]],
            'bx', mew=10)
    # plot trajectory of hand
    ax.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2])
    # plot trajectory of target
    ax.plot([target_xyz[0]], [target_xyz[1]], [target_xyz[2]],
            'rx', mew=10)
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

    # create a target based on initial arm position
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', q=feedback['q'])
    target_xyz = start + np.array([-.25, .25, 0])
    interface.set_xyz(name='target', xyz=target_xyz)

    count = 0.0
    while 1:  # count < 1500:
        # get arm feedback from VREP
        feedback = interface.get_feedback()

        # use visual feedback to get object endpoint position
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])

        # generate control signal
        u = ctrlr.control(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_xyz,
            target_vel=np.zeros(3))

        print('error: ', np.sqrt(np.sum((target_xyz - ee_xyz)**2)))
        # apply the control signal, step the sim forward
        interface.apply_u(u)

        # set orientation of hand object to match EE
        quaternion = robot_config.orientation('EE', q=feedback['q'])
        angles = abr_control.utils.transformations.euler_from_quaternion(
            quaternion, axes='rxyz')
        interface.set_orientation('hand', angles)

        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target_xyz))

        count += 1

finally:
    # stop and reset the VREP simulation
    interface.disconnect()
