"""
Running sliding control with nonlinear adaptation using the
PyGame display. The controller works to drive the arm's end-effector to
the target while an unexpected external force is applied. Target position
can be by clicking inside the display.
To turn adaptation on or off, press the spacebar.
"""
import numpy as np
import pygame

from abr_control.arms import threejoint as arm
# from abr_control.arms import twojoint as arm
from abr_control.interfaces import PyGame
from abr_control.controllers import Sliding, signals


# initialize our robot config
robot_config = arm.Config(use_cython=True)
# get Jacobians to each link for calculating perturbation
J_links = [robot_config._calc_J('link%s' % ii, x=[0, 0, 0])
           for ii in range(robot_config.N_LINKS)]

# create our arm simulation
arm_sim = arm.ArmSim(robot_config)

# create an operational space controller
ctrlr = Sliding(robot_config, kd=20)

# create our nonlinear adaptation controller
adapt = signals.DynamicsAdaptation(
    n_input=robot_config.N_JOINTS,
    n_output=robot_config.N_JOINTS,
    pes_learning_rate=1e-2,
    )


def on_click(self, mouse_x, mouse_y):
    self.target[0] = self.mouse_x
    self.target[1] = self.mouse_y


def on_keypress(self, key):
    if key == pygame.K_SPACE:
        self.adaptation = not self.adaptation
        print('adaptation: ', self.adaptation)


# create our interface
interface = PyGame(robot_config, arm_sim,
                   on_click=on_click,
                   on_keypress=on_keypress)
interface.connect()
interface.adaptation = False  # set adaptation False to start

# create a target
feedback = interface.get_feedback()
target_xyz = robot_config.Tx('EE', feedback['q'])
interface.set_target(target_xyz)


try:
    print('\nSimulation starting...')
    print('Click to move the target.')
    print('Press space to turn on adaptation.\n')

    count = 0
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        # generate an operational space control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target_xyz)

        # if adaptation is on (toggled with space bar)
        if interface.adaptation:
            sig = adapt.generate(
                input_signal=robot_config.scaledown('q', feedback['q']),
                training_signal=-ctrlr.s)
            u += sig

        fake_gravity = np.array([[0, -9.81, 0, 0, 0, 0]]).T * 10.0
        g = np.zeros((robot_config.N_JOINTS, 1))
        for ii in range(robot_config.N_LINKS):
            pars = tuple(feedback['q']) + tuple([0, 0, 0])
            g += np.dot(J_links[ii](*pars).T, fake_gravity)
        u += g.squeeze()

        new_target = interface.get_mousexy()
        if new_target is not None:
            target_xyz[:2] = new_target
        interface.set_target(target_xyz)

        # apply the control signal, step the sim forward
        interface.send_forces(
            u, update_display=True if count % 20 == 0 else False)

        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')
