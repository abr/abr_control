"""
Running the threelink arm with the pygame display, using the
obstacle avoidance signal used to avoid self collision.
The target location can be moved by clicking on the background.
"""
import numpy as np

import abr_control

print('\nClick to move the target.\n')

# initialize our robot config for the ur5
robot_config = abr_control.arms.threelink.config(use_cython=True)

# create our environment
interface = abr_control.interfaces.maplesim(
    robot_config, dt=.001, on_click_move='obstacle')
interface.connect()

ctrlr = abr_control.controllers.osc(
    robot_config, kp=100, vmax=10)
avoid = abr_control.controllers.signals.avoid_obstacles(
    robot_config, threshold=1)

# create an obstacle
interface.display.add_circle(xyz=[0, 0, 0], radius=.2)

# create a target
target_xyz = [0, 2, 0]
interface.set_target(target_xyz)

# set up lists for tracking data
ee_path = []
target_path = []

print('\nClick to move the obstacle.\n')
try:
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        # generate an operational space control signal
        u = ctrlr.control(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_xyz)
        # add in obstacle avoidance
        obs_x, obs_y = interface.display.get_mousexy()
        u += avoid.generate(
            q=feedback['q'],
            obstacles=[[obs_x, obs_y, 0, .2]])

        target_xyz[0], target_xyz[1] = interface.display.get_mousexy()
        interface.set_target(target_xyz)

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # track data
        ee_path.append(np.copy(hand_xyz))
        target_path.append(np.copy(target_xyz))

finally:
    # stop and reset the simulation
    interface.disconnect()
