"""
A basic script for connecting and moving the arm to 4 targets.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position.
"""
import numpy as np
import signal
import sys

import abr_control

# initialize our robot config for neural controllers
robot_config = abr_control.arms.jaco2.config(
    regenerate_functions=True, use_cython=True,
    hand_attached=False)
# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.joint(robot_config, kp=4, kv=2)

ctrlr.control(np.zeros(robot_config.num_joints),
              np.zeros(robot_config.num_joints),
              np.zeros(robot_config.num_joints))

# create our interface for the jaco2
interface = abr_control.interfaces.vrep(
    robot_config=robot_config, dt=.001)

target_pos = np.array([2.0, 2.75, 3.45, 1.0, .5], dtype='float32')
target_vel = None

# connect to the jaco
interface.connect()

# set up arrays for tracking end-effector and target position
q_track = []
ctr = 0


def on_exit(signal, frame):
    """ A function for plotting the end-effector trajectory and error """
    global q_track
    q_track = np.array(q_track)

    import matplotlib.pyplot as plt

    plt.plot(q_track)
    plt.plot(np.ones(q_track.shape) *
                ((target_pos + np.pi) % (np.pi * 2) - np.pi),
                'r--')
    plt.tight_layout()
    plt.show()
    sys.exit()

# call on_exit when ctrl-c is pressed
signal.signal(signal.SIGINT, on_exit)

try:
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', q=feedback['q'])
    while 1:
        ctr += 1
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', q=feedback['q'])

        u = ctrlr.control(q=feedback['q'], dq=feedback['dq'],
                          target_pos=target_pos, target_vel=target_vel)
        interface.send_forces(np.array(u, dtype='float32'))

        print('q: ', feedback['q'])
        # set orientation of hand object to match EE
        quaternion = robot_config.orientation('EE', q=feedback['q'])
        angles = abr_control.utils.transformations.euler_from_quaternion(
            quaternion, axes='rxyz')
        interface.set_orientation('hand', angles)

        q_track.append(np.copy(feedback['q']))

except Exception as e:
    print(e)

finally:
    # close the connection to the arm
    interface.disconnect()

        
