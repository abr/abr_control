"""
Running operational space control using Mujoco. The controller will
move the end-effector to the target object's position and orientation.

This example controls all 6 degrees of freedom (position and orientation),
and applies a second order path planner to both position and orientation targets

After termination the script will plot results
"""
import numpy as np
import glfw

from abr_control.controllers import OSC, Damping, path_planners
from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.interfaces.mujoco import Mujoco
from abr_control.utils import transformations


# initialize our robot config
robot_config = arm('jaco2')

# create our interface
interface = Mujoco(robot_config, dt=.001)
interface.connect()
interface.send_target_angles(robot_config.START_ANGLES)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create opreational space controller
ctrlr = OSC(
    robot_config,
    kp=30,  # position gain
    kv=20,
    ko=180,  # orientation gain
    null_controllers=[damping],
    vmax=None,  # [m/s, rad/s]
    # control all DOF [x, y, z, alpha, beta, gamma]
    ctrlr_dof = [True, True, True, True, True, True])

feedback = interface.get_feedback()
hand_xyz = robot_config.Tx('EE', feedback['q'])

def get_target(robot_config):
    # pregenerate our path and orientation planners
    n_timesteps = 2000
    traj_planner = path_planners.SecondOrderDMP(
        error_scale=0.01, n_timesteps=n_timesteps)

    starting_orientation = robot_config.quaternion('EE', feedback['q'])

    target_orientation = np.random.random(3)
    target_orientation /= np.linalg.norm(target_orientation)
    # convert our orientation to a quaternion
    target_orientation = [0] + list(target_orientation)

    #target_position = [0.4, -0.3, 0.5]
    mag = 0.6
    target_position = np.random.random(3)*0.5
    target_position = target_position / np.linalg.norm(target_position) * mag

    traj_planner.generate_path(
        position=hand_xyz, target_pos=target_position)
    _, orientation_planner = traj_planner.generate_orientation_path(
        orientation=starting_orientation, target_orientation=target_orientation)

    return traj_planner, orientation_planner, target_position, target_orientation

# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []


try:
    count = 0

    print('\nSimulation starting...\n')
    while 1:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])
        if count % 3000 == 0:
            traj_planner, orientation_planner, target_position, target_orientation = get_target(robot_config)
            interface.set_mocap_xyz('target_orientation', target_position)
            interface.set_mocap_orientation('target_orientation', target_orientation)

        pos, vel = traj_planner.next()
        orient = orientation_planner.next()
        target = np.hstack([pos, orient])

        interface.set_mocap_xyz('path_planner_orientation', target[:3])
        interface.set_mocap_orientation('path_planner_orientation',
            transformations.quaternion_from_euler(
                orient[0], orient[1], orient[2], 'rxyz'))

        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            #target_vel=np.hstack([vel, np.zeros(3)])
            )

        # add gripper forces
        u = np.hstack((u, np.ones(3)*0.05))

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

    ee_track = np.array(ee_track).T
    ee_angles_track = np.array(ee_angles_track).T
    target_track = np.array(target_track).T
    target_angles_track = np.array(target_angles_track).T

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611
        label_pos = ['x', 'y', 'z']
        label_or = ['a', 'b', 'g']
        c = ['r', 'g', 'b']

        fig = plt.figure(figsize=(8,12))
        ax1 = fig.add_subplot(311)
        ax1.set_ylabel('3D position (m)')
        for ii, ee in enumerate(ee_track):
            ax1.plot(ee, label='EE: %s' % label_pos[ii], c=c[ii])
            ax1.plot(target_track[ii], label='Target: %s' % label_pos[ii],
                     c=c[ii], linestyle='--')
        ax1.legend()

        ax2 = fig.add_subplot(312)
        for ii, ee in enumerate(ee_angles_track):
            ax2.plot(ee, label='EE: %s' % label_or[ii], c=c[ii])
            ax2.plot(target_angles_track[ii], label='Target: %s'%label_or[ii],
                     c=c[ii], linestyle='--')
        ax2.set_ylabel('3D orientation (rad)')
        ax2.set_xlabel('Time (s)')
        ax2.legend()

        ee_track = ee_track.T
        target_track = target_track.T
        ax3 = fig.add_subplot(313, projection='3d')
        ax3.set_title('End-Effector Trajectory')
        ax3.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label='ee_xyz')
        ax3.plot(target_track[:, 0], target_track[:, 1], target_track[:, 2],
                 label='ee_xyz', c='g', linestyle='--')
        ax3.scatter(target_track[-1, 0], target_track[-1, 1],
                    target_track[-1, 2], label='target', c='g')
        ax3.legend()
        plt.show()
