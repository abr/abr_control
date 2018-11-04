"""
Running operational space control with the PyGame display. The arm will
move the end-effector to a target orientation, which can be changed by
pressing the left/right arrow keys.
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
ctrlr = OSC(robot_config, kp=50)
# create our arm simulation
arm_sim = arm.ArmSim(robot_config)

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
                   on_keypress=on_keypress)
interface.connect()
interface.theta = -3 * np.pi / 4
feedback = interface.get_feedback()
R = robot_config.R('EE', feedback['q'])
interface.on_keypress(interface, None)

# set up lists for tracking data
ee_path = []

# control (gamma) out of [x, y, z, alpha, beta, gamma]
ctrlr_dof = [False, False, False, False, False, True]


try:
    print('\nSimulation starting...')
    print('Press left or right arrow to change target orientation angle.\n')

    count = 0
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        target = np.hstack([np.zeros(3), interface.target_angles])
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            ctrlr_dof=ctrlr_dof,
            )

        # apply the control signal, step the sim forward
        interface.send_forces(
            u, update_display=True if count % 20 == 0 else False)

        # track data
        ee_path.append(np.copy(hand_xyz))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')
