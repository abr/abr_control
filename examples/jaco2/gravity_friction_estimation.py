"""
Implementing the gravity and friction estimation as
described in (Liu and Quach, 2001).
"""
import numpy as np
import sympy as sp
from sympy.utilities.autowrap import autowrap

import abr_control

# initialize our robot config
robot_config = abr_control.arms.jaco2.config_link3(
    regenerate_functions=True)

# create our VREP interface
interface = abr_control.interfaces.vrep(
    robot_config, dt=.001)

target_positions = [
    robot_config.rest_angles + np.ones(robot_config.num_joints) * .2,
    robot_config.rest_angles - np.ones(robot_config.num_joints) * .2]

kp = np.diag([1, 1, 1, 1, 1, 1])
kv = np.diag([1, 1, 1, 1, 1, 1])


def gravity_function(use_cython=False):
    # create the upper triangle W matrix where each element is
    # wij(q) = g.T (dR_0^j/ dq_i) p_j
    # where g = [0, 0, 9.8062].T, R_0^j is the rotation matrix
    # from the origin to joint j, and p_j is the direction in frame i
    # pointing to the centre of mass m_i of link i
    gravity = sp.Matrix([0, 0, 9.8062])  # gravity
    p = sp.Matrix([1, 0, 0])  # COM direction is always along x

    W = []
    for ii in range(robot_config.num_joints):
        W.append([])
        for jj in range(robot_config.num_joints):
            if jj < ii:
                W[ii].append(0)
            else:
                R0j = robot_config._calc_T('joint%i' % jj)[:3, :3]
                # fill in the upper triangle
                W[ii].append(
                    (gravity.T * R0j.diff(robot_config.q[ii]) * p)[0, 0])
    W = sp.Matrix(W)
    print(W)
    if use_cython is True:
        return autowrap(W, backend="cython", args=robot_config.q)
    return sp.lambdify(robot_config.q, "numpy")

Wfunc = gravity_function()

# try:
#     interface.connect()
#     # create a target based on initial arm position
#     feedback = interface.get_feedback()
#
#     W = np.zeros((2, robot_config.num_joints, robot_config.num_joints))
#     e = np.zeros((2, robot_config.num_joints))
#     for ii, target_pos in enumerate(target_positions):
#         print('Moving to target ', ii+1)
#         stable_count = 0
#         # move to stable state
#         while 1:
#             # get arm feedback from VREP
#             feedback = interface.get_feedback()
#
#             u = np.dot(
#                 kp, target_pos - feedback['q']) - np.dot(kv, feedback['dq'])
#             # apply the control signal, step the sim forward
#             interface.apply_u(u)
#
#             # set orientation of hand object to match EE
#             quaternion = robot_config.orientation('EE', q=feedback['q'])
#             angles = abr_control.utils.transformations.euler_from_quaternion(
#                 quaternion, axes='rxyz')
#             interface.set_orientation('hand', angles)
#
#             if np.sum(feedback['dq']**2) < .05:
#                 # if the arm has reach a stable state, start counting
#                 stable_count += 1
#                 if stable_count > 500:
#                     # assume now it's actually a stable state
#                     break
#                 if stable_count % 10 == 0:
#                     print('stable count: ', stable_count)
#             else:
#                 stable_count = 0
#         # from stable state, calculate gravity basis function
#         W[ii] = gravity_function(feedback['q'])
#         # calculate error between desired state and stable state
#         e[ii] = target_pos - feedback['q']
#         print('Data collected')
#
#     # calculate gravity parameters
#     W = W[0] - W[1]
#     Wstar = np.dot(W.T, np.linalg.pinv(np.dot(W, W.T)))
#     theta_g = np.dot(Wstar, np.dot(kp, e[0] - e[1]))
#     # calculate friction parameters
#     theta_f = np.dot(kp, e[0]) - np.dot(W[0], theta_g)
#
# finally:
#     # stop and reset the VREP simulation
#     interface.disconnect()
