"""
A basic script for connecting and moving the arm to 4 targets.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position.
"""
import numpy as np
import time

import abr_control

# initialize our robot config for neural controllers
robot_config = abr_control.arms.jaco2.config.robot_config()
# instantiate the REACH controller for the ur5 robot
ctrlr = abr_control.controllers.osc_robust.controller(
  robot_config, kp=600, vmax=0.35)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
ctrlr.control(np.zeros(6), np.zeros(6), target_state=np.zeros(6))

# create our VREP interface for the ur5
interface = abr_control.interfaces.jaco2.interface(robot_config)
# connect to the jaco
interface.connect()
# move to the home position
interface.apply_q(robot_config.home_position)
# switch to torque control mode
interface.init_force_mode()

# set up arrays for tracking end-effector and target position
ee_track = []
targets_track = []

count = 0
target_index = 0
at_target_count = 0

# list of targets to move to
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
        interface.apply_u(np.array(u, dtype='float32'))
        comm_speed.append(time.time() - start_comm)

        hand_xyz = robot_config.T('EE', q=q)
        error = np.sqrt(np.sum((hand_xyz - target_xyz)**2))
        if error < .01:
            # if we're at the target, start count
            # down to moving to the next target
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
        targets_track.append(target_xyz)
        count += 1

except Exception as e:
    print(e)

finally:
    # return back to home position
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    # close the connection to the arm
    interface.disconnect()

    if count > 0:  # i.e. if it successfully ran
        import matplotlib.pyplot as plt
        import seaborn

        ee_track = np.array(ee_track)
        targets_track = np.array(targets_track)
        # plot targets and trajectory of end-effectory in 3D
        abr_control.utils.plotting.plot_trajectory(ee_track, targets_track)

        plt.tight_layout()
        plt.show()
