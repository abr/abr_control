"""
Running the threelink arm with the pygame display. The arm will
move the end-effector to the target, which can be moved by
clicking on the background.
"""
import numpy as np

from abr_control.arms import threejoint as arm
# from abr_control.arms import twojoint as arm
from abr_control.interfaces import PyGame
from abr_control.controllers import OSC, Damping, RestingConfig


# initialize our robot config
robot_config = arm.Config(use_cython=True)
# create our arm simulation
arm_sim = arm.ArmSim(robot_config)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# keep the arm near a default configuration
resting_config = RestingConfig(
    robot_config, kp=50, kv=np.sqrt(50),
    rest_angles=[np.pi/4, np.pi/4, None])

# create an operational space controller
ctrlr = OSC(robot_config, kp=20, use_C=True, null_controllers=[damping])


def on_click(self, mouse_x, mouse_y):
    self.target[0] = self.mouse_x
    self.target[1] = self.mouse_y


# create our interface
interface = PyGame(robot_config, arm_sim, dt=.001, on_click=on_click)
interface.connect()

# create a target
feedback = interface.get_feedback()
target_xyz = robot_config.Tx('EE', feedback['q'])
interface.set_target(target_xyz)

# set up lists for tracking data
ee_path = []
target_path = []


try:
    # run ctrl.generate once to load all functions
    zeros = np.zeros(robot_config.N_JOINTS)
    ctrlr.generate(q=zeros, dq=zeros, target_pos=np.zeros(3))
    robot_config.R('EE', q=zeros)

    print('\nSimulation starting...\n')
    print('\nClick to move the target.\n')

    count = 0
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        # generate an operational space control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_xyz,
            target_vel=np.zeros(3))

        new_target = interface.get_mousexy()
        if new_target is not None:
            target_xyz[0:2] = new_target
        interface.set_target(target_xyz)

        # apply the control signal, step the sim forward
        interface.send_forces(
            u, update_display=True if count % 50 == 0 else False)

        # track data
        ee_path.append(np.copy(hand_xyz))
        target_path.append(np.copy(target_xyz))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')
