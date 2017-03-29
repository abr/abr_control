"""
A basic script for connecting and moving the arm to 4 targets
is joint space. The joint angles are recorded and plotted against
the target angles once the final target is reached, and the arm
has moved back to its default resting position.
"""
import numpy as np
import sys
import traceback

import abr_control

# initialize our robot config for neural controllers
robot_config = abr_control.arms.jaco2.config(
    use_cython=True, hand_attached=True)
# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.joint(robot_config, kp=20, kv=6)

zeros = np.zeros(robot_config.num_joints)
ctrlr.control(zeros, zeros, zeros)

# create our interface for the jaco2
interface = abr_control.interfaces.vrep(
    robot_config=robot_config, dt=.001)

target_pos = np.array([2.0, 1.4, 1.8, 1.0, .5, .6], dtype='float32')
target_vel = None

# connect to the jaco
interface.connect()

# set up arrays for tracking end-effector and target position
q_track = []


try:
    while 1:
        feedback = interface.get_feedback()

        u = ctrlr.control(q=feedback['q'], dq=feedback['dq'],
                          target_pos=target_pos, target_vel=target_vel)
        interface.send_forces(np.array(u, dtype='float32'))

        q_track.append(np.copy(feedback['q']))

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.disconnect()

    q_track = np.array(q_track)

    import matplotlib.pyplot as plt

    plt.plot(q_track)
    plt.gca().set_color_cycle(None)
    plt.plot(np.ones(q_track.shape) *
             ((target_pos + np.pi) % (np.pi * 2) - np.pi), '--')
    plt.legend(range(6))
    plt.tight_layout()
    plt.show()
    sys.exit()
