"""
Move the UR5 VREP arm to a target position while avoiding an obstacle.
The simulation ends after 1.5 simulated seconds, and the trajectory
of the end-effector is plotted in 3D.
"""
import numpy as np

from abr_control.arms import ur5 as arm
# from abr_control.arms import jaco2 as arm
from abr_control.controllers import OSC, signals
from abr_control.interfaces import VREP

# initialize our robot config
robot_config = arm.Config(use_cython=True)
# if using the Jaco 2 arm with the hand attached, use the following instead:
# robot_config = arm.Config(use_cython=True, hand_attached=False)

# instantiate the REACH controller with obstacle avoidance
ctrlr = OSC(robot_config, kp=200, vmax=0.5)
avoid = signals.AvoidObstacles(robot_config)

# create our VREP interface
interface = VREP(robot_config, dt=.001)
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

    target_xyz = start + np.array([.2, -.2, 0.0])
    interface.set_xyz(name='target', xyz=target_xyz)

    moving_obstacle = True
    obstacle_xyz = np.array([0.09596, -0.3661, 0.64204])
    interface.set_xyz(name='obstacle', xyz=obstacle_xyz)

    # run ctrl.generate once to load all functions
    zeros = np.zeros(robot_config.N_JOINTS)
    ctrlr.generate(q=zeros, dq=zeros, target_pos=target_xyz)
    robot_config.R('EE', q=zeros)

    print('\nSimulation starting...\n')

    count = 0.0
    obs_count = 0.0
    while count < 1500:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()
        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_xyz,
            target_vel=np.zeros(3))

        # get obstacle position from VREP
        obs_x, obs_y, obs_z = interface.get_xyz('obstacle')
        # update avoidance system about obstacle position
        avoid.set_obstacles([[obs_x, obs_y, obs_z, 0.05]])
        if moving_obstacle is True:
            obs_x = .125 + .25 * np.sin(obs_count)
            obs_count += .01
            interface.set_xyz(name='obstacle',
                              xyz=[obs_x, obs_y, obs_z])

        # add in obstacle avoidance control signal
        u += avoid.generate(q=feedback['q'])

        # send forces into VREP, step the sim forward
        interface.send_forces(u)

        # calculate end-effector position
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])
        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target_xyz))
        obstacle_track.append(np.copy([obs_x, obs_y, obs_z]))

        count += 1

finally:
    # stop and reset the VREP simulation
    interface.disconnect()

    print('Simulation terminated...')

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)
    obstacle_track = np.array(obstacle_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from abr_control.utils.plotting import plot_3D

        plt.figure()
        plt.plot(np.sqrt(np.sum((np.array(target_track) -
                                np.array(ee_track))**2, axis=1)))
        plt.ylabel('Distance (m)')
        plt.xlabel('Time (ms)')
        plt.title('Distance to target')

        ax = plot_3D(ee_track, target_track)
        # plot trajectory of obstacle
        ax.plot(obstacle_track[:, 0], obstacle_track[:, 1],
                obstacle_track[:, 2], 'r--', mew=10)
        plt.show()
