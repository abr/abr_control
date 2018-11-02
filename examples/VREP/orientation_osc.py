"""
Running the threelink arm with the pygame display. The arm will
move the end-effector to the target, which can be moved by
clicking on the background.
"""
import numpy as np
import pygame

from abr_control.arms import ur5 as arm
from abr_control.controllers import OSC
from abr_control.interfaces import VREP
from abr_control.utils import transformations


# initialize our robot config
robot_config = arm.Config(use_cython=True)

# create our interface
interface = VREP(robot_config, dt=.005)
interface.connect()

try:

    # set up lists for tracking data
    ee_path = []
    target_path = []

    kw = 0.0  # gain on angular velocity difference
    ko = 100.0  # gain on orientation error
    kv = 10.0
    w_d = 0.0  # target angular velocity
    dw_d = 0.0  # target angular acceleration

    # run ctrl.generate once to load all functions
    zeros = np.zeros(robot_config.N_JOINTS)
    robot_config.R('EE', q=zeros)

    print('\nSimulation starting...\n')

    count = 0
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        # get orientation of target from VREP
        euler_angles = interface.get_orientation('target')
        # transform into a quaternion
        q_target = transformations.quaternion_from_euler(
            euler_angles[0], euler_angles[1], euler_angles[2], axes='sxyz')

        # from (Yuan, 1988), given r = [r1, r2, r3]
        # r^x = [[0, -r3, r2], [r3, 0, -r1], [-r2, r1, 0]]
        q_target_matrix = np.array([
            [0, -q_target[2], q_target[1]],
            [q_target[2], 0, -q_target[0]],
            [-q_target[1], q_target[0], 0]])

        # get orientation in quaternion form for the end effector
        # in origin frame coordinates, calculated from the rotation matrix
        R = robot_config.R('EE', feedback['q'])
        q_EE = transformations.quaternion_from_matrix(R)

        J = robot_config.J('EE', feedback['q'])[3:]
        w = np.dot(J, feedback['q'])  # calculate the task space velocities

        # calculate the difference between q_EE and q_target
        # from (Yuan, 1988)
        # dq = (w_d * [x, y, z] - w * [x_d, y_d, z_d] -
        #       [x_d, y_d, z_d]^x * [x, y, z])
        w_tilde = - ko * (q_target[0] * q_EE[1:] - q_EE[0] * q_target[1:] -
                          np.dot(q_target_matrix, q_EE[1:]))

        M = robot_config.M(feedback['q'])
        M_inv = np.linalg.inv(M)
        Mx_inv = np.dot(J, np.dot(M_inv, J.T))
        if np.linalg.det(Mx_inv) != 0:
            Mx = np.linalg.inv(Mx_inv)
        else:
            Mx = np.linalg.pinv(Mx_inv, rcond=0.005)

        u = np.dot(J.T, np.dot(Mx, w_tilde)) - kv * np.dot(M, feedback['dq'])
        u -= robot_config.g(feedback['q'])

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # track data
        ee_path.append(np.copy(hand_xyz))
        # target_path.append(np.copy(target_xyz))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')
