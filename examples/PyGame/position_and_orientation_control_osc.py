"""
Running operational space control with the PyGame display. The arm will
move the end-effector to the target at a specified orientation around
the z axis. The (x, y) target can be changed by clicking on the background,
and the target orientation can be changed with the left/right arrow keys.
"""
import numpy as np
import pygame

from abr_control.arms import threejoint as arm
# from abr_control.arms import twojoint as arm
from abr_control.interfaces import PyGame
from abr_control.utils import transformations
from abr_control.controllers import OSC


# initialize our robot config
robot_config = arm.Config(use_cython=True)
# create opreational space controller
ctrlr = OSC(robot_config, kp=50)
# create our arm simulation
arm_sim = arm.ArmSim(robot_config)


def on_click(self, mouse_x, mouse_y):
    self.target[0] = self.mouse_x
    self.target[1] = self.mouse_y


def on_keypress(self, key):
    if key == pygame.K_LEFT:
        self.theta += np.pi / 10
    if key == pygame.K_RIGHT:
        self.theta -= np.pi / 10
    print('theta: ', self.theta)

    # set the target orientation to be the initial EE
    # orientation rotated by theta
    R_theta = np.array([
        [np.cos(interface.theta), -np.sin(interface.theta), 0],
        [np.sin(interface.theta), np.cos(interface.theta), 0],
        [0, 0, 1]])
    R_target = np.dot(R_theta, R)
    self.target_angles = transformations.euler_from_matrix(R_target,
                                                           axes='sxyz')


# create our interface
interface = PyGame(robot_config, arm_sim, dt=.001,
                   on_click=on_click, on_keypress=on_keypress)
interface.connect()
feedback = interface.get_feedback()
# set target position
target_xyz = robot_config.Tx('EE', feedback['q'])
interface.set_target(target_xyz)
# set target orientation
interface.theta = -3 * np.pi / 4
R = robot_config.R('EE', feedback['q'])
interface.on_keypress(interface, None)

# set up lists for tracking data
ee_path = []
target_path = []

# control (x, y, gamma) out of [x, y, z, alpha, beta, gamma]
ctrlr_dof = [True, True, False, False, False, True]


try:
    print('\nSimulation starting...')
    print('Click to move the target.')
    print('Press left or right arrow to change target orientation angle.\n')

    count = 0
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        target = np.hstack([target_xyz, interface.target_angles])
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            ctrlr_dof=ctrlr_dof,
            )

        new_target = interface.get_mousexy()
        if new_target is not None:
            target_xyz[0:2] = new_target
        interface.set_target(target_xyz)

        # apply the control signal, step the sim forward
        interface.send_forces(
            u, update_display=True if count % 20 == 0 else False)

        # track data
        ee_path.append(np.copy(hand_xyz))
        target_path.append(np.copy(target_xyz))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')
