"""
A basic script for connecting to the arm and putting it in floating
mode, which only compensates for gravity. The end-effector position
is recorded and plotted when the script is exited (with ctrl-c).

In this example, the floating controller is applied in the joint space
"""
import numpy as np
import traceback
import glfw

from abr_control.controllers import Floating, RestingConfig
from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.interfaces.mujoco import Mujoco
from abr_control.utils import transformations


rest_angles=[None, 1.57, 2.2, None, None, 3.14]

# initialize our robot config
robot_config = arm('jaco2')

# create the Mujoco interface and connect up
interface = Mujoco(robot_config, dt=.001)
interface.connect()
interface.send_target_angles(robot_config.START_ANGLES)

# instantiate the controller
ctrlr = Floating(robot_config, task_space=True, dynamic=True)
resting = RestingConfig(
    robot_config,
    rest_angles=rest_angles,
    kp=12)

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

        u_rest = resting.generate(
            q=feedback['q'],
            dq=feedback['dq'])

        u += u_rest

        # add gripper forces
        u = np.hstack((u, np.ones(3)*0.05))

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
    q_track = np.asarray(q_track).T

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        col = ['r', 'g', 'b', 'y', 'm', 'k']
        fig = plt.figure(figsize=(8,12))
        ax1 = fig.add_subplot(111)
        ax1.set_title('Joint Angles')
        ax1.set_ylabel('Angle (rad)')
        ax1.set_xlabel('Time (ms)')
        for ii, q in enumerate(q_track):
            if rest_angles[ii] is not None:
                ax1.plot(q_track[ii], label='joint %i' % ii, c=col[ii])
                ax1.plot(np.ones(len(q_track.T)) * rest_angles[ii], c=col[ii], linestyle='--')
        plt.legend()
        plt.show()
