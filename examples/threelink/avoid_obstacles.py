"""
Running the threelink arm with the PyGame display, using the
obstacle avoidance signal. The obstacle location can be moved
by clicking on the background.
"""
import numpy as np

import abr_control
from abr_control.arms.threelink.arm_sim import ArmSim
from abr_control.interfaces.pygame import PyGame

# initialize our robot config for the ur5
robot_config = abr_control.arms.threelink.Config(use_cython=True)
# create our arm simulation
arm_sim = ArmSim(robot_config)

def on_click(self, mouse_x, mouse_y):
    self.circles[0][0] = mouse_x
    self.circles[0][1] = mouse_y

# create our interface
interface = PyGame(robot_config, arm_sim,
                   on_click=on_click)
interface.connect()

ctrlr = abr_control.controllers.OSC(
    robot_config, kp=20, vmax=10)
avoid = abr_control.controllers.signals.AvoidObstacles(
    robot_config, threshold=1)

# create an obstacle
interface.add_circle(xyz=[0, 0, 0], radius=.2)

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
        # add in obstacle avoidance
        obs_xy = interface.get_mousexy()
        if obs_xy is not None:
            avoid.set_obstacles(obstacles=[[obs_xy[0], obs_xy[1], 0, .2]])
        u += avoid.generate(q=feedback['q'])

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # change target location once hand is within
        # 5mm of the target
        if (np.sqrt(np.sum((target_xyz - hand_xyz)**2)) < .005):
            target_xyz = np.array([
                np.random.random() * 2 - 1,
                np.random.random() * 2 + 1,
                0])
            # update the position of the target
            interface.set_target(target_xyz)

        # track data
        ee_path.append(np.copy(hand_xyz))
        target_path.append(np.copy(target_xyz))

finally:
    # stop and reset the simulation
    interface.disconnect()
