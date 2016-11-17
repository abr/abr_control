import numpy as np

import abr_control

# initialize our robot config for the ur5
robot_config = abr_control.arms.threelink.config.robot_config(
    regenerate_functions=False)

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
    num_targets = -1
    while num_targets < len(targets_xyz):
        # get arm feedback from VREP
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('joint1', feedback['q'])
        # generate a control signal
        u = ctrlr.control(
            q=feedback['q'],
            dq=feedback['dq'],
            target_state=np.hstack(
                [target_xyz, np.zeros(3)]))
        print('error: ', np.sqrt(np.sum((target_xyz - hand_xyz)**2)))
        # apply the control signal, step the sim forward
        interface.apply_u(u)
        print('(x, y): ', interface.display.get_mousexy())

        # change target location once hand is within
        # 5mm of the target
        if (num_targets < 0 or
                np.sqrt(np.sum((target_xyz - hand_xyz)**2)) < .005):
            target_xyz = targets_xyz[num_targets]
            # update the position of the target sphere in VREP
            interface.set_target(target_xyz)
            num_targets += 1
            print('Reaching to target %i' % num_targets)
            print(target_xyz)

        # track data
        ee_path.append(np.copy(hand_xyz))
        target_path.append(np.copy(target_xyz))

finally:
    # stop and reset the VREP simulation
    interface.disconnect()
    # # generate a 3D plot of the trajectory taken
    # abr_control.utils.plotting.plot_trajectory(
    #     ee_path=ee_path,
    #     target_path=target_path)
    #
    # import matplotlib.pyplot as plt
    # plt.plot(np.sqrt(np.sum((np.array(target_path) -
    #                          np.array(ee_path))**2, axis=1)))
    # plt.ylabel('Error (m)')
    # plt.xlabel('Time (ms)')
    # plt.show()
