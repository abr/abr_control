"""
A basic script for connecting and moving the arm to several target.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position.
"""
import numpy as np

import abr_control

# initialize our robot config for neural controllers
robot_config = abr_control.arms.onelink.config.robot_config()
# instantiate the REACH controller for the onelink robot
ctrlr = abr_control.controllers.osc.controller(
    robot_config, kp=600)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
ctrlr.control(np.zeros(1), np.zeros(1), target_state=np.zeros(6))

# create our VREP interface for the onelink arm
interface = abr_control.interfaces.vrep.interface(robot_config)
# connect to the jaco
interface.connect()

# set up arrays for tracking end-effector and target position
ee_track = []
targets_track = []

count = 0
target_index = 0
at_target_count = 0

# list of targets to move to
targets = [[.3, 0.0, .375],
           [-.3, 0.0, .375]]

def set_target(xyz):
    # normalize target position to lie on path of arm's end-effector
    xyz = xyz / np.linalg.norm(xyz) * robot_config.L[1, 0] + robot_config.L[0]
    print('target xyz: ', xyz)
    interface.set_xyz('target', xyz)
    return xyz

# normalize target position to lie on path of arm's end-effector
target_xyz = set_target(targets[0])
print('Moving to first target: ', target_xyz)

try:
    while 1:
        feedback = interface.get_feedback()
        q = feedback['q']
        dq = feedback['dq']
        u = ctrlr.control(q=q, dq=dq,
                          target_state=np.hstack([target_xyz,
                                                 np.zeros(3)]))
        interface.apply_u(np.array(u, dtype='float32'))

        hand_xyz = robot_config.Tx('EE', q=q)
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
                    target_xyz = set_target(targets[target_index])
                    print('Moving to next target: ', target_xyz)
                at_target_count = 0

        ee_track.append(hand_xyz)
        targets_track.append(target_xyz)
        count += 1

except Exception as e:
    print(e)

finally:
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
