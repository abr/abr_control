"""
Move the UR5 VREP arm to a target position while avoiding an obstacle.
The simulation ends after 1.5 simulated seconds, and the trajectory
of the end-effector is plotted in 3D.
"""
import numpy as np

import abr_control

# initialize our robot config for the ur5
robot_config = abr_control.arms.ur5.config(
    regenerate_functions=False)

# instantiate the REACH controller with obstacle avoidance
ctrlr = abr_control.controllers.osc(
    robot_config, kp=200, vmax=0.5)
avoid = abr_control.controllers.signals.avoid_obstacles(
    robot_config)

# create our VREP interface
interface = abr_control.interfaces.vrep(
    robot_config, dt=.001)
interface.connect()

# set up lists for tracking data
ee_track = []
target_track = []
obstacle_track = []

try:
    num_targets = 0
    back_to_start = False

    # get visual position of end point of object
    feedback = interface.get_feedback()
    # set up the values to be used by the Jacobian for the object end effector
    start = robot_config.Tx('EE', q=feedback['q'])

    target_xyz = start + np.array([.25, -.25, 0.0])
    interface.set_xyz(name='target', xyz=target_xyz)

    moving_obstacle = True
    obstacle_xyz = np.array([0.09596, -0.3661, 0.64204])
    interface.set_xyz(name='obstacle', xyz=obstacle_xyz)

    count = 0.0
    obs_count = 0.0
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

        # locate obstacle
        obs_x, obs_y, obs_z = interface.get_xyz('obstacle')
        if moving_obstacle is True:
            obs_x = .125 + .25 * np.sin(obs_count)
            obs_count += .01
            interface.set_xyz(name='obstacle', xyz=[obs_x, obs_y, obs_z])

        # add in obstacle avoidance control signal
        u += avoid.generate(
            q=feedback['q'],
            obstacles=[[obs_x, obs_y, obs_z, 0.05]])

        print('error: ', np.sqrt(np.sum((target_xyz - ee_xyz)**2)))
        # apply the control signal, step the sim forward
        interface.apply_u(u)

        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target_xyz))
        obstacle_track.append(np.copy([obs_x, obs_y, obs_z]))

        count += 1

finally:
    # stop and reset the VREP simulation
    interface.disconnect()

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)
    obstacle_track = np.array(obstacle_track)

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
    # plot trajectory of obstacle
    ax.plot(obstacle_track[:, 0],
            obstacle_track[:, 1],
            obstacle_track[:, 2],
            'yx', mew=10)

    plt.figure()
    plt.plot(np.sqrt(np.sum((np.array(target_track) -
                             np.array(ee_track))**2, axis=1)))
    plt.ylabel('Error (m)')
    plt.xlabel('Time (ms)')
    plt.show()
