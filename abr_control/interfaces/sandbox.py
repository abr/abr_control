import numpy as np
import time

import abr_control

# initialize our robot config for neural controllers
robot_config = abr_control.arms.jaco2.config.robot_config()
# instantiate the REACH controller for the ur5 robot
ctrlr = abr_control.controllers.osc_robust.controller(
	robot_config, kp=600, vmax=0.35)
# run controller once to generate functions / take care of overhead
# outside of the main loop (so the torque mode isn't exited)
ctrlr.control(np.zeros(6), np.zeros(6), target_state=np.zeros(6))
# create our VREP interface for the ur5
interface = abr_control.interfaces.jaco2.interface(robot_config)
interface.connect()

q_track = []
dq_track = []
ee_track = []
count = 0
target_index = 0
at_target_count = 0

targets = [[-.467, .22, .78],
           [-.467, -.22, .78],
           [.467, -.22, .78],
           [.467, .22, .78],
           [-.467, .22, .78]]
target_xyz = targets[0]
print('Moving to first target: ', target_xyz)

try:
    while 1:
        feedback = interface.get_feedback()
        q = (np.array(feedback['q']) % 360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0
        u = ctrlr.control(q=q, dq=dq,
                          target_state=np.hstack([target_xyz,
                                                 np.zeros(3)]))
        #print('u: ', u)
        interface.apply_u(np.array(u, dtype='float32'))

        hand_xyz = robot_config.T('EE', q=q)
        error = np.sqrt(np.sum((hand_xyz - target_xyz)**2))
        print('error: ', error)
        if error < .01:
            at_target_count += 1
            if at_target_count >= 200:
                target_index += 1
                if target_index > len(targets):
                    break
                else:
                    target_xyz = targets[target_index]
                    print('Moving to next target: ', target_xyz)
                at_target_count = 0

        ee_track.append(hand_xyz)
        q_track.append(q)
        dq_track.append(dq)
        count += 1

finally:
    interface.disconnect()

    if count > 0:
        import matplotlib.pyplot as plt
        import seaborn

        plt.subplot(2, 1, 1)
        plt.plot(q_track)
        plt.legend(range(6))
        plt.title('Joint angles')

        plt.subplot(2, 1, 2)
        plt.plot(dq_track[10:])
        plt.legend(range(6))
        plt.title('Joint velocities')

        ee_track = np.array(ee_track)
        targets_plot = np.ones(ee_track.shape) * target_xyz
        for ii in range(len(targets)):
            targets_plot[ii] = targets[ii]
        abr_control.utils.plotting.plot_trajectory(ee_track, targets_plot)

        plt.tight_layout()
        plt.show()
