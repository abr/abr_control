"""
A basic script for connecting and moving the arm to a target
configuration in joint space.
"""
import numpy as np
import traceback

# from abr_control.arms import jaco2 as arm
from abr_control.arms import ur5 as arm
from abr_control.controllers import Joint
from abr_control.interfaces import VREP

# initialize our robot config for neural controllers
robot_config = arm.Config(use_cython=True)

# instantiate the REACH controller for the jaco2 robot
ctrlr = Joint(robot_config, kp=20, kv=6)

zeros = np.zeros(robot_config.N_JOINTS)
ctrlr.generate(zeros, zeros, zeros)

# create interface and connect
interface = VREP(robot_config=robot_config, dt=.001)
interface.connect()

target_pos = np.array([2.0, 1.4, 1.8, 1.0, .5, .6], dtype='float32')

# get joint handles for shadow
names = ['joint%i_shadow' % ii for ii in range(robot_config.N_JOINTS)]
joint_handles = []
for name in names:
    interface.get_xyz(name)  # this loads in the joint handle
    joint_handles.append(interface.misc_handles[name])
# move shadow to target position
interface.send_target_angles(target_pos, joint_handles)

# set up arrays for tracking end-effector and target position
q_track = []


try:
    while 1:
        feedback = interface.get_feedback()

        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_pos)
        interface.send_forces(np.array(u, dtype='float32'))

        q_track.append(np.copy(feedback['q']))

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.disconnect()

    q_track = np.array(q_track)
    if q_track.shape[0] > 0:
        import matplotlib.pyplot as plt
        plt.plot((q_track + np.pi) % (np.pi * 2) - np.pi)
        plt.gca().set_color_cycle(None)
        plt.plot(np.ones(q_track.shape) *
                ((target_pos + np.pi) % (np.pi * 2) - np.pi), '--')
        plt.legend(range(robot_config.N_LINKS))
        plt.tight_layout()
        plt.show()
