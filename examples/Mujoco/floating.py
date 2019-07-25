"""
A basic script for connecting to the arm and putting it in floating
mode, which only compensates for gravity. The end-effector position
is recorded and plotted when the script is exited (with ctrl-c).

In this example, the floating controller is applied in the joint space
"""
import numpy as np
import traceback
import glfw

from abr_control.controllers import Floating
from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.interfaces.mujoco import Mujoco
from abr_control.utils import transformations


# initialize our robot config
robot_config = arm('jaco2')

# instantiate the controller
ctrlr = Floating(robot_config, task_space=True, dynamic=True)

# create the Mujoco interface and connect up
interface = Mujoco(robot_config, dt=.001)
interface.connect()
interface.send_target_angles(robot_config.START_ANGLES)

# set up arrays for tracking end-effector and target position
ee_track = []
q_track = []


try:
    # get the end-effector's initial position
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', q=feedback['q'])

    print('\nSimulation starting...\n')

    while 1:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()

        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'])

        # send forces into Mujoco
        interface.send_forces(u)

        # calculate the position of the hand
        hand_xyz = robot_config.Tx('EE', q=feedback['q'])
        # track end effector position
        ee_track.append(np.copy(hand_xyz))
        q_track.append(np.copy(feedback['q']))

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
