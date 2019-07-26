"""
Running operational space control using Mujoco. The controller will
move the end-effector to the target object's X position and orientation.

The cartesian direction being controlled is set in the first three booleans
of the ctrlr_dof parameter
"""
import numpy as np
import glfw

from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.interfaces.mujoco import Mujoco
from abr_control.utils import transformations
from abr_control.controllers import OSC, Damping

print('******************************')
print('* DOF Controlled: x, a, b ,g *')
print('******************************')
# initialize our robot config
robot_config = arm('jaco2')
ctrlr_dof = [False, True, False, True, True, True]
dof_labels = ['x', 'y', 'z', 'a', 'b', 'g']

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create opreational space controller
ctrlr = OSC(
    robot_config,
    kp=30,
    kv=20,
    ko=180,
    null_controllers=[damping],
    vmax=[10, 10],  # [m/s, rad/s]
    # control (x, alpha, beta, gamma) out of [x, y, z, alpha, beta, gamma]
    ctrlr_dof=ctrlr_dof)

# create our interface
interface = Mujoco(robot_config, dt=.001)
interface.connect()

target_orientation = np.random.random(4)
target_orientation /= np.linalg.norm(target_orientation)
target_xyz = np.array([0.3, 0.3, 0.5])

# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []


try:
    count = 0
    print('\nSimulation starting...\n')
    interface.set_mocap_xyz('target_orientation', target_xyz)
    interface.set_mocap_orientation('target_orientation', target_orientation)
    while 1:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        target = np.hstack([
            interface.get_mocap_xyz('target_orientation'),
            transformations.euler_from_quaternion(
                interface.get_mocap_orientation('target_orientation'), 'rxyz')])

        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            )

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # track data
        ee_track.append(np.copy(hand_xyz))
        ee_angles_track.append(transformations.euler_from_matrix(
            robot_config.R('EE', feedback['q']), axes='rxyz'))
        target_track.append(np.copy(target[:3]))
        target_angles_track.append(np.copy(target[3:]))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')

    ee_track = np.array(ee_track)
    ee_angles_track = np.array(ee_angles_track)
    target_track = np.array(target_track)
    target_angles_track = np.array(target_angles_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        fig = plt.figure(figsize=(8,12))
        ax1 = fig.add_subplot(311)
        ax1.set_ylabel('3D position (m)')
        for ii, controlled_dof in enumerate(ctrlr_dof[:3]):
            if controlled_dof:
                ax1.plot(ee_track[:, ii], label=dof_labels[ii])
                ax1.plot(target_track[:, ii], '--')
        ax1.legend()

        ax2 = fig.add_subplot(312)
        for ii, controlled_dof in enumerate(ctrlr_dof[3:]):
            if controlled_dof:
                ax2.plot(ee_angles_track[:, ii], label=dof_labels[ii+3])
                ax2.plot(target_angles_track[:, ii], '--')
        ax2.set_ylabel('3D orientation (rad)')
        ax2.set_xlabel('Time (s)')
        ax2.legend()

        ax3 = fig.add_subplot(313, projection='3d')
        ax3.set_title('End-Effector Trajectory')
        ax3.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label='ee_xyz')
        ax3.scatter(target_track[0, 0], target_track[0, 1], target_track[0, 2],
                    label='target', c='g')
        ax3.legend()
        plt.show()
