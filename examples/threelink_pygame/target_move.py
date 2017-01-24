"""
Running the threelink arm with the pygame display. The arm will
move the end-effector to the target, which can be moved by
clicking on the background.
"""
import numpy as np

import abr_control

# initialize our robot config for the ur5
robot_config = abr_control.arms.threelink.config(
    regenerate_functions=False)

# create an operational space controller
ctrlr = abr_control.controllers.osc(
    robot_config, kp=100, vmax=10)

# create our interface
interface = abr_control.interfaces.maplesim(
    robot_config, dt=.001, on_click_move='target')
interface.connect()

# create a target
feedback = interface.get_feedback()
target_xyz = robot_config.Tx('EE', feedback['q'])
interface.set_target(target_xyz)

# set up lists for tracking data
ee_path = []
target_path = []

print('\nClick to move the target.\n')
try:
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        # generate an operational space control signal
        u = ctrlr.control(
            q=feedback['q'],
            dq=feedback['dq'],
            target_x=target_xyz)

        target_xyz[0], target_xyz[1] = interface.display.get_mousexy()
        interface.set_target(target_xyz)

        # apply the control signal, step the sim forward
        interface.apply_u(u)

        # track data
        ee_path.append(np.copy(hand_xyz))
        target_path.append(np.copy(target_xyz))

finally:
    # stop and reset the simulation
    interface.disconnect()
