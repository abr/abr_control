"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1.5 simulated seconds, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np
import sys
import traceback

import abr_control

# initialize our robot config for the ur5
robot_config = abr_control.arms.jaco2.config(use_cython=True)

# instantiate controller
ctrlr = abr_control.controllers.osc(
    robot_config, kp=20, kv=4, vmax=1,
    null_control=True)

# create our VREP interface
interface = abr_control.interfaces.vrep(
    robot_config, dt=.001)
interface.connect()

# set up lists for tracking data
ee_track = []
target_track = []


try:
    num_targets = 0
    back_to_start = False

    # create a target based on initial arm position
    feedback = interface.get_feedback()
    target_xyz = np.array(interface.get_xyz('target'))
    print('target_xyz: ', target_xyz)
    T_inv = robot_config.T_inv('EE', q=feedback['q'])
    offset = np.dot(
        T_inv, np.hstack([interface.get_xyz('tooltip'), 1]))[:-1]
    print('offset: ', offset)

    interface.set_xyz(name='target', xyz=target_xyz)

    count = 0.0
    while count < 2000:  # time in dt specified in interface init
        # get arm feedback from VREP
        feedback = interface.get_feedback()

        # generate control signal
        u = ctrlr.control(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_xyz,
            offset=offset)

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # use visual feedback to get object endpoint position
        xyz = robot_config.Tx('EE', q=feedback['q'], x=offset)
        # track data
        ee_track.append(np.copy(xyz))
        target_track.append(np.copy(target_xyz))

        count += 1

except:
    print(traceback.format_exc())

finally:
    # stop and reset the VREP simulation
    interface.disconnect()

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # plot start point of hand
    ax.plot([ee_track[0, 0]], [ee_track[0, 1]], [ee_track[0, 2]],
            'bx', mew=10)
    # plot trajectory of hand
    ax.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2])
    # plot trajectory of target
    ax.plot(target_track[:, 0], target_track[:, 1], target_track[:, 2],
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
