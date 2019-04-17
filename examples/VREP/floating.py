"""
A basic script for connecting to the arm and putting it in floating
mode, which only compensates for gravity. The end-effector position
is recorded and plotted when the script is exited (with ctrl-c).
"""
import numpy as np
import traceback

from abr_control.arms import ur5 as arm
# from abr_control.arms import jaco2 as arm
# from abr_control.arms import onelink as arm
from abr_control.controllers import Floating
from abr_control.interfaces import VREP

# initialize our robot config
robot_config = arm.Config(use_cython=True)
# if using the Jaco 2 arm with the hand attached, use the following instead:
# robot_config = arm.Config(use_cython=True, hand_attached=True)

# instantiate the controller
ctrlr = Floating(robot_config, dynamic=True)

# create the VREP interface and connect up
interface = VREP(robot_config, dt=.005)
interface.connect()

# set up arrays for tracking end-effector and target position
ee_track = []
q_track = []


try:
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', q=feedback['q'])

    # run ctrl.generate once to load all functions
    zeros = np.zeros(robot_config.N_JOINTS)
    ctrlr.generate(q=zeros, dq=zeros)
    robot_config.R('EE', q=zeros)

    print('\nSimulation starting...\n')
    while 1:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()
        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'])
        # send forces into VREP
        interface.send_forces(u)

        # calculate the position of the hand
        hand_xyz = robot_config.Tx('EE', q=feedback['q'])
        # track end effector position
        ee_track.append(hand_xyz)
        q_track.append(feedback['q'])

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.disconnect()

    print('Simulation terminated...')

    ee_track = np.array(ee_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        fig = plt.figure(figsize=(8,12))
        ax1 = fig.add_subplot(211)
        ax1.set_title('Joint Angles')
        ax1.set_ylabel('Angle (rad)')
        ax1.set_xlabel('Time (ms)')
        ax1.plot(q_track)
        ax1.legend()

        ax2 = fig.add_subplot(212, projection='3d')
        ax2.set_title('End-Effector Trajectory')
        ax2.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label='ee_xyz')
        ax2.legend()
        plt.show()
