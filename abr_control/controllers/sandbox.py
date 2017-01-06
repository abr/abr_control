import numpy as np
import time

import abr_control

# # initialize our robot config for the ur5
# robot_config = abr_control.arms.threelink.config.robot_config(
#     regenerate_functions=True)
#
# # test out our orientation calculations
# q = np.array([1, 1, 1])
# print(robot_config.orientation('EE', q=q))

# initialize our robot config for the ur5
# robot_config = abr_control.arms.ur5.config.robot_config(


robot_config = abr_control.arms.onelink.config.robot_config(
    regenerate_functions=True)

interface = abr_control.interfaces.vrep.interface(
    robot_config=robot_config, dt=.001)
interface.connect()

try:
    # test out our orientation calculations
    while 1:
        name = 'EE'
        feedback = interface.get_feedback()
        print(feedback['q'])
        xyz = robot_config.Tx(name, q=feedback['q'],
                              x=[.1, .12, .025])
        print('xyz: ', xyz)
        angles = robot_config.orientation(name, q=feedback['q'])
        print('angles: ', np.array(angles).T * 180 / np.pi)

        interface.set_xyz('hand', xyz)
        interface.set_orientation('hand', angles)
        time.sleep(1)

finally:
    # stop and reset the VREP simulation
    interface.disconnect()

# # # alpha = 180 * np.pi / 180.0
# # # beta = -90 * np.pi / 180.0
# # # gamma = -90 * np.pi / 180.0
# alpha = 0 * np.pi / 180.0
# beta = 0 * np.pi / 180.0
# gamma = 90 * np.pi / 180.0
#
# Rx = np.array([
#     [np.cos(alpha), -np.sin(alpha), 0],
#     [np.sin(alpha), np.cos(alpha), 0],
#     [0, 0, 1]])
#
# Ry = np.array([
#     [np.cos(beta), 0, -np.sin(beta)],
#     [0, 1, 0],
#     [np.sin(beta), 0, np.cos(beta)]])
#
# Rz = np.array([
#     [1, 0, 0],
#     [0, np.cos(gamma), -np.sin(gamma)],
#     [0, np.sin(gamma), np.cos(gamma)]])
#
# ans = np.dot(Rz, np.dot(Ry, Rx))
# ans[ans < 1e-5] = 0
# print(ans)
