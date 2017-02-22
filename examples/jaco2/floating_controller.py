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
robot_config = abr_control.arms.jaco2.config_link5(
    use_cython=True, hand_attached=False)
# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.floating(
    robot_config)

robot_config.generate_control_functions()

# create our interface for the jaco2
interface = abr_control.interfaces.vrep(
    robot_config=robot_config, dt=.001)
# connect to the jaco
interface.connect()

# set up arrays for tracking end-effector and target position
ee_track = []


def on_exit(signal, frame):
    """ A function for plotting the end-effector trajectory and error """
    global ee_track
    ee_track = np.array(ee_track)

    import matplotlib.pyplot as plt

    abr_control.utils.plotting.plot_trajectory(
        ee_track, np.zeros(ee_track.shape))

    plt.tight_layout()
    plt.show()
    sys.exit()

# call on_exit when ctrl-c is pressed
signal.signal(signal.SIGINT, on_exit)

try:
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', q=feedback['q'])
    while 1:
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', q=feedback['q'])

        u = ctrlr.control(q=feedback['q'], dq=feedback['dq'],)
        interface.send_forces(np.array(u, dtype='float32'))

        # set orientation of hand object to match EE
        quaternion = robot_config.orientation('EE', q=feedback['q'])
        angles = abr_control.utils.transformations.euler_from_quaternion(
            quaternion, axes='rxyz')
        interface.set_orientation('hand', angles)

        ee_track.append(hand_xyz)

except Exception as e:
    print(e)

finally:
    # close the connection to the arm
    interface.disconnect()

        
