"""
Running the threelink arm with the pygame display. The arm will
move the end-effector to the target, which can be moved by
clicking on the background.
"""
import numpy as np

from abr_control.arms import threejoint as arm
# from abr_control.arms import twojoint as arm
from abr_control.interfaces import PyGame
from abr_control.controllers import OSC
from abr_control.utils import transformations


# initialize our robot config
robot_config = arm.Config(use_cython=True)
# create our arm simulation
arm_sim = arm.ArmSim(robot_config)

# create an operational space controller
ctrlr = OSC(robot_config, kp=20, vmax=None,
            use_C=True, use_g=False)


def on_click(self, mouse_x, mouse_y):
    self.target[0] = self.mouse_x
    self.target[1] = self.mouse_y


# create our interface
interface = PyGame(robot_config, arm_sim, dt=.001, on_click=on_click)
interface.connect()

# create a target position
feedback = interface.get_feedback()
target_xyz = robot_config.Tx('EE', feedback['q']) - np.array([-.5, 1, 0])
print(target_xyz)
interface.set_target(target_xyz)

# target orientation is initial orientation + theta rotation around z axis
R = robot_config.R('EE', feedback['q'])
theta = - 3 * np.pi / 4
R_theta = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta), np.cos(theta), 0],
    [0, 0, 1]])
R_target = np.dot(R_theta, R)
q_target = transformations.quaternion_from_matrix(R_target)

# from (Yuan, 1988), given r = [r1, r2, r3]
# r^x = [[0, -r3, r2], [r3, 0, -r1], [-r2, r1, 0]]
q_target_matrix = np.array([
    [0, -q_target[2], q_target[1]],
    [q_target[2], 0, -q_target[0]],
    [-q_target[1], q_target[0], 0]])

# set up lists for tracking data
ee_path = []
target_path = []


kw = 0.0  # gain on angular velocity difference
kp = 10.0  # gain or position error
ko = 10.0  # gain on orientation error
kv = 10.0
w_d = 0.0  # target angular velocity
dw_d = 0.0  # target angular acceleration
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

        # first step is to get the quaternion for the end effector
        # in origin frame coordinates, calculated from the rotation matrix
        R = robot_config.R('EE', feedback['q'])
        q_EE = transformations.quaternion_from_matrix(R)

        J = robot_config.J('EE', feedback['q'])

        # calculate the difference between q_EE and q_target
        # from (Yuan, 1988)
        # dq = (w_d * [x, y, z] - w * [x_d, y_d, z_d] -
        #       [x_d, y_d, z_d]^x * [x, y, z])
        w_tilde = - ko * (q_target[0] * q_EE[1:] - q_EE[0] * q_target[1:] -
                np.dot(q_target_matrix, q_EE[1:]))
        x_tilde = - kp * (hand_xyz - target_xyz)

        forces = np.hstack([x_tilde, w_tilde])[:, None]

        J = np.vstack([J[0], J[1], J[5]])
        forces = np.vstack([forces[0], forces[1], forces[5]]).squeeze()

        M = robot_config.M(feedback['q'])
        M_inv = np.linalg.inv(M)
        Mx_inv = np.dot(J, np.dot(M_inv, J.T))
        if np.linalg.det(Mx_inv) != 0:
            Mx = np.linalg.inv(Mx_inv)
        else:
            Mx = np.linalg.pinv(Mx_inv, rcond=0.05)

        u = np.dot(J.T, np.dot(Mx, forces)) - kv * np.dot(M, feedback['dq'])

        new_target = interface.get_mousexy()
        if new_target is not None:
            target_xyz[0:2] = new_target
        interface.set_target(target_xyz)

        # apply the control signal, step the sim forward
        interface.send_forces(
            u, update_display=True if count % 20 == 0 else False)

        # track data
        ee_path.append(np.copy(hand_xyz))
        # target_path.append(np.copy(target_xyz))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')
