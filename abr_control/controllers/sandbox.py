import numpy as np

import abr_control

# initialize our robot config for the ur5
# robot_config = abr_control.arms.ur5.config.robot_config(


robot_config = abr_control.arms.onelink.config.robot_config(
    regenerate_functions=True)

# create our VREP interface
interface = abr_control.interfaces.maplesim.interface(
    robot_config, dt=.001)
interface.connect()

# # instantiate the REACH controller
np.random.seed(17)
obstacles = [
    [.5, 2, 0, .2],
    [-1, .6, 0, .2]]
for obstacle in obstacles:
    # add obstacle to display
    interface.display.add_circle(
        xyz=obstacle[:3], radius=obstacle[3])

ctrlr = abr_control.controllers.osc_obstacles.controller(
    robot_config, kp=100, vmax=10, obstacles=obstacles, threshold=.5)

# create a target
targets_xyz = [
    [0, 2, 0],
    [.5, 2, 0],
    [-.5, 2, 0]]
target_xyz = targets_xyz[0]

# set up lists for tracking data
ee_path = []
target_path = []

try:
    # test out our orientation calculations
    while 1:
        name = 'EE'
        feedback = interface.get_feedback()
        print(feedback['q'])
        xyz = robot_config.Tx(name, q=feedback['q'],
                              x=[.025, .12, -.1])

        print('xyz: ', xyz)
        angles = robot_config.orientation(name, q=feedback['q'])
        print('angles: ', np.array(angles).T * 180 / np.pi)
        # angles = np.array([0, 80, 90]) * np.pi / 180

        interface.set_xyz('hand', xyz)
        interface.set_orientation('hand', angles)
        # print(interface.get_orientation('hand'))
        # interface.set_orientation('hand', interface.get_orientation('Disc')[1])
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
