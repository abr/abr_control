"""
Running the threelink arm with the PyGame display, using an exponential
additive signal when to push away from joints and  avoid self collision.
The target location can be moved by clicking on the background.
"""
import numpy as np

import abr_control
from abr_control.arms.threelink.arm_sim import ArmSim
from abr_control.interfaces.pygame import PyGame

print('\nClick to move the target.\n')

# initialize our robot config for the ur5
robot_config = abr_control.arms.threelink.Config(use_cython=True)
# create our arm simulation
arm_sim = ArmSim(robot_config)

def on_click(self, mouse_x, mouse_y):
    self.target[0] = self.mouse_x
    self.target[1] = self.mouse_y

# create our interface
interface = PyGame(robot_config, arm_sim,
    dt=.001, on_click=on_click,
    q_init=[np.pi/4, np.pi/2, np.pi/2])
interface.connect()

ctrlr = abr_control.controllers.OSC(
    robot_config, kp=100, vmax=10)
avoid = abr_control.controllers.signals.AvoidJointLimits(
    robot_config,
    min_joint_angles=[np.pi/5, np.pi/5.0, np.pi/4.0],
    max_joint_angles=[np.pi/2, 4*np.pi/5.0, 3*np.pi/4.0],
    max_torque=[10, 100, 1000])

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
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_xyz)
        # add in joint limit avoidance
        u += avoid.generate(feedback['q'])

        new_target = interface.get_mousexy()
        if new_target is not None:
            target_xyz[:2] = new_target
        interface.set_target(target_xyz)

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # track data
        ee_path.append(np.copy(hand_xyz))
        target_path.append(np.copy(target_xyz))

finally:
    # stop and reset the simulation
    interface.disconnect()
