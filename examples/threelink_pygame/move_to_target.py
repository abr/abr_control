"""
Running the threelink arm with the pygame display. The arm will
move the end-effector to the target, which can be moved by
clicking on the background.
"""
import numpy as np

import abr_control
from abr_control.interfaces.maplesim import MapleSim

print('\nClick to move the target.\n')

# initialize our robot config for the ur5
robot_config = abr_control.arms.threelink.Config(use_cython=True)

# create an operational space controller
ctrlr = abr_control.controllers.OSC(
    robot_config, kp=100, vmax=10)

def on_click(self, mouse_x, mouse_y):
    self.target[0] = self.mouse_x
    self.target[1] = self.mouse_y

# create our interface
interface = MapleSim(robot_config, dt=.001,
                     on_click=on_click)
interface.connect()

# create a target
feedback = interface.get_feedback()
target_xyz = robot_config.Tx('EE', feedback['q'])
interface.set_target(target_xyz)

# set up lists for tracking data
ee_path = []
target_path = []

try:
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        # generate an operational space control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_xyz)

        new_target = interface.display.get_mousexy()
        if new_target is not None:
            target_xyz[0:2] = new_target
        interface.set_target(target_xyz)

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # track data
        ee_path.append(np.copy(hand_xyz))
        target_path.append(np.copy(target_xyz))

finally:
    # stop and reset the simulation
    interface.disconnect()
