"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1.5 simulated seconds, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np

import abr_control

# initialize our robot config for the ur5
robot_config = abr_control.arms.ur5.config(
    regenerate_functions=False)

# instantiate controller
ctrlr = abr_control.controllers.osc(
    robot_config, kp=600, vmax=None)

# create our VREP interface
interface = abr_control.interfaces.vrep.interface(
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
    start = robot_config.Tx('EE', q=feedback['q'])
    target_xyz = start + np.array([.25])
    interface.set_xyz(name='target', xyz=target_xyz)

    count = 0.0
    while count < 1500:
        # get arm feedback from VREP
        feedback = interface.get_feedback()

        # use visual feedback to get object endpoint position
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])

        # generate control signal
        u = ctrlr.control(
            q=feedback['q'],
            dq=feedback['dq'],
            target_state=np.hstack((
                target_xyz,
                [0, 0, 0])))

        print('error: ', np.sqrt(np.sum((target_xyz - ee_xyz)**2)))
        # apply the control signal, step the sim forward
        interface.apply_u(u)

        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target_xyz))

        count += 1

finally:
    # stop and reset the VREP simulation
    interface.disconnect()

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # plot start point of hand
    ax.plot([ee_track[0, 0]],
            [ee_track[0, 1]],
            [ee_track[0, 2]],
            'bx', mew=10)
    # plot trajectory of hand
    ax.plot(ee_track[:, 0],
            ee_track[:, 1],
            ee_track[:, 2])
    # plot trajectory of target
    ax.plot(target_track[:, 0],
            target_track[:, 1],
            target_track[:, 2],
            'rx', mew=10)

    plt.figure()
    plt.plot(np.sqrt(np.sum((np.array(target_track) -
                             np.array(ee_track))**2, axis=1)))
    plt.ylabel('Error (m)')
    plt.xlabel('Time (ms)')
    plt.show()
