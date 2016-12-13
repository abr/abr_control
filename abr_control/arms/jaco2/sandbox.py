import numpy as np

import abr_control

# initialize our robot config for neural controllers
robot_config = abr_control.arms.jaco2.config.robot_config(regenerate_functions=False)
# instantiate the REACH controller for the ur5 robot
ctrlr = abr_control.controllers.osc.controller(robot_config, kp=1150, kv=60, vmax=0.25)
# create our VREP interface for the ur5
interface = abr_control.interfaces.vrep.interface(robot_config, dt=0.001)
interface.connect()

# set up lists for tracking data
ee_track = []
target_track = []
obstacle_track = []
at_target_count = 0
try:
    num_targets = 0
    back_to_start = False

    # get visual position of end point of object
    feedback = interface.get_feedback()
    # set up the values to be used by the Jacobian for the object end effector
    start = robot_config.Tx('EE', q=feedback['q'])

    target_xyz = start + np.array([.25, -.25, -.2])
    interface.set_xyz(name='target', xyz=target_xyz)

    #obstacle_xyz = np.array([0.1746, -0.2161, 0.66704])  # avoidance 1
    # obstacle_xyz = np.array([-0.1254, -0.3411, 0.34204])  # avoidance 2
    # obstacle_xyz = np.array([0.09596, -0.3661, 0.64204])  # avoidance 3 and 4
    #interface.set_xyz(name='obstacle', xyz=obstacle_xyz)
    #moving_obstacle = False  # set True for avoidance 4

    count = 0.0
    while 1:
        # get arm feedback from VREP
        feedback = interface.get_feedback()

        # use visual feedback to get object endpoint position
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])

        # generate control signal
        u = ctrlr.control(
            q=feedback['q'],
            dq=feedback['dq'],
            target_state=np.hstack((
                target_xyz,
                [0, 0, 0])))

        error = np.sqrt(np.sum((target_xyz - ee_xyz)**2))
        print('error: ', error)
        # apply the control signal, step the sim forward
        interface.apply_u(u)

        if error < .01:
            at_target_count += 1
            if at_target_count >= 200:
                print('T A R G E T  R E A C H E D')
                break        
        
        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target_xyz))

        count += 1

finally:
    # stop and reset the VREP simulation
    interface.disconnect()

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    #np.savez_compressed('data/3.1/ee', ee=ee_track)
    #np.savez_compressed('data/3.1/targets', targets=target_track)

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # plot start point of hand
    ax.plot([ee_track[0, 0]],
            [ee_track[0, 1]],
            [ee_track[0, 2]],
            'bx', mew=10)
    # plot trajectory of hand
    ax.plot(ee_track[:, 0],
            ee_track[:, 1],
            ee_track[:, 2])
    # plot trajectory of target
    ax.plot(target_track[:, 0],
            target_track[:, 1],
            target_track[:, 2],
            'rx', mew=10)

    plt.figure()
    plt.show()
