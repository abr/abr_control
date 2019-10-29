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
robot_config = arm('jaco2_gripper')

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create opreational space controller
ctrlr = OSC(
    robot_config,
    kp=100,  # position gain
    kv=20,
    ko=180,  # orientation gain
    null_controllers=[damping],
    vmax=None,  # [m/s, rad/s]
    # control all DOF [x, y, z, alpha, beta, gamma]
    ctrlr_dof = [True, True, True, True, True, True])

# create our interface
interface = Mujoco(robot_config, dt=.001)
interface.connect()

feedback = interface.get_feedback()
hand_xyz = robot_config.Tx('EE', feedback['q'])

def gripper_forces(command=None, grip_force=3):
    if command == 'open':
        u_gripper = np.ones(3) * grip_force
    elif command == 'close':
        u_gripper = np.ones(3) * -grip_force
    elif command is None:
        u_gripper = np.zeros(3)
    return u_gripper

def _get_approach(target_pos, approach_buffer=0.03, z_offset=0):
    """
    Takes the target location, and returns an
    orientation to approach the target, along with a target position that
    is approach_buffer meters short of the target, with a z offset
    determined by z_offset in meters. The orientation is set to be a vector
    that would connect the robot base to the target, with the gripper parallel
    to the ground

    Parameters
    ----------
    target_pos: list of 3 floats
        xyz cartesian loc of target of interest [meters]
    approach_buffer: float, Optional (Default: 0.03)
        we want to approach the target along the orientation that connects
        the base of the arm to the target, but we want to stop short before
        going for the grasp. This variable sets that distance to stop short
        of the target [meters]
    z_offset: float, Optional (Default: 0.2)
        sometimes it is desirable to approach a target from above or below.
        This gets added to the final target position
    """
    # save a copy of the target in case weird pointer things happen with lists
    # target_z = np.copy(target_pos[2])
    target_pos = np.asarray(target_pos)


    # get signs of target directions so our approach target stays in the same
    # octant as the provided target
    # target_sign = target_pos / abs(target_pos)

    dist_to_target = np.linalg.norm(target_pos)
    approach_vector = np.copy(target_pos)
    approach_vector /= np.linalg.norm(approach_vector)
    # print('get_approach target_pos: ', target_pos)

    approach_pos = approach_vector * (dist_to_target - approach_buffer)
    #approach_vector[2] = 0

    # world z pointing up, rotate by pi/2 to be parallel with ground
    theta1 = np.pi/2
    q1 = [np.cos(theta1/2),
        0,
        np.sin(theta1/2),
        0
        ]
    # now we rotate about z to get x pointing up
    theta2 = np.arctan2(target_pos[1], target_pos[0])
    # print('theta2: ', theta2)
    q2 = [np.cos(theta2/2),
        0,
        0,
        np.sin(theta2/2),
        ]


    # get our combined rotation
    q3 = transformations.quaternion_multiply(q2, q1)

    # interface.set_xyz('magenta', approach_pos)
    # interface.set_orientation('magenta',
    #         transformations.euler_from_quaternion(
    #             q3, 'rxyz'))

    approach_pos[2] += z_offset

    return approach_pos, q3


def get_approach_path(
        robot_config, path_planner, q, target_pos, max_reach_dist=None,
        min_z=0, target_orientation=None, start_pos=None, **kwargs):
    """
    Accepts a robot config, path planner, and target_position, returns the
    generated position and orientation paths to approach the target for grasping

    Parameters
    ----------
    robot_config: instantiated abr_control/arms/base_config.py subclass
        used to determine the current arms orientation and position
    path_planner: instantiated
        abr_control/controllers/path_planners/path_planner.py subclass
        used to filter the path to the final target, and to generate the
        orientation path with the same reaching profile
    q: list of floats
        the current joint possitions of the arm [radians]
    target_pos: list of 3 floats, Optional (Default: None)
        cartesian location of final target [meters], if None a random one will
        be generated
    max_reach_dist: float, Optional (Default: None)
        the maximum distance [meters] from origin the arm can reach. The target
        is normalized and the target is set along that vector, max_reach_dist
        from the origin. If None, the target is left as is
    target_orientation: list of three floats, Optional (Default: None)
        euler angles for target orientation, leave as None to automatically
        calculate the orientation for a grasping approach
        (see _get_approach() )
    start_pos: list of three floats, Optional (Default: None)
        if left as None will use the current EE position
    """

    if target_pos[2] < min_z:
        # make sure the target isn't too close to the ground
        target_pos[2] = min_z

    if max_reach_dist is not None:
        # normalize to make sure our target is within reaching distance
        target_pos = target_pos / np.linalg.norm(target_pos) * max_reach_dist

    # get our EE starting orientation and position
    starting_R = robot_config.R('EE', feedback['q'])
    starting_orientation = transformations.quaternion_from_matrix(
        starting_R)
    if start_pos is None:
        start_pos = robot_config.Tx('EE', q)

    # calculate our target approach position and orientation
    approach_pos, approach_orient = _get_approach(
        target_pos=target_pos, **kwargs)

    if target_orientation is not None:
        print('Using manual target orientation')
        approach_orient = target_orientation

    # generate our path to our approach position
    path_planner.generate_path(
        position=start_pos, target_pos=approach_pos)

    # generate our orientation planner
    _, orientation_planner = path_planner.generate_orientation_path(
        orientation=starting_orientation,
        target_orientation=approach_orient)
    target_data = {
                'target_pos': target_pos,
                'approach_pos': approach_pos,
                'approach_orient': approach_orient}

    return path_planner, orientation_planner, target_data


def second_order_path_planner(n_timesteps=2000, error_scale=1):
    """
    Define your path planner of choice here
    """
    traj_planner = path_planners.BellShaped(
        error_scale=error_scale, n_timesteps=n_timesteps)
    return traj_planner


# def arc_path_planner(n_timesteps=2000, **kwargs):
#     """
#     Define your path planner of choice here
#     """
#
#     traj_planner = path_planners.FirstOrderArc(n_timesteps=n_timesteps)
#     return traj_planner

def target_shift(interface, scale=0.01):
    """
    Parameters
    ----------
    scale: float, optional (Default: 0.01)
        the amount to move with each button press [meters]
    """
    shifted_target = scale * np.array([
        interface.viewer.target_x,
        interface.viewer.target_y,
        interface.viewer.target_z])

    interface.viewer.target_x = 0
    interface.viewer.target_y = 0
    interface.viewer.target_z = 0

    return shifted_target

# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []


try:
    object_xyz = np.array([0, 0.5, 0.3])
    deposit_xyz = np.array([0.4, 0.5, 0.3])
    reach_list = [
             # move above object
             {'target_pos': object_xyz,
              'start_pos': None,
              'orientation': None,
              'n_timesteps': 2000,
              'grasp_command': 'open',
              'hold_timesteps': None,
              'z_offset': 0.2,
              'approach_buffer': 0.02,
              'traj_planner': second_order_path_planner
             },
             # get into grasping position
             {'target_pos': object_xyz,
              'start_pos': None,
              'orientation': None,
              'n_timesteps': 1000,
              'grasp_command': 'open',
              'hold_timesteps': None,
              'z_offset': 0,
              'approach_buffer': 0,
              'traj_planner': second_order_path_planner
             },
             # grasp object
             {'target_pos': object_xyz,
              'start_pos': None,
              'orientation': None,
              'n_timesteps': 1000,
              'grasp_command': 'close',
              'hold_timesteps': 500,
              'z_offset': 0,
              'approach_buffer': 0,
              'traj_planner': second_order_path_planner
             },
             # lift object
             {'target_pos': object_xyz,
              'start_pos': None,
              'orientation': None,
              'n_timesteps': 1000,
              'grasp_command': 'close',
              'hold_timesteps': None,
              'z_offset': 0.2,
              'approach_buffer': 0.03,
              'traj_planner': second_order_path_planner
             },
             # move to above drop off
             {'target_pos': deposit_xyz,
              'start_pos': None,
              'orientation': None,
              'n_timesteps': 1000,
              'grasp_command': 'close',
              'hold_timesteps': None,
              'z_offset': 0.2,
              'approach_buffer': 0.0,
              'traj_planner': second_order_path_planner
             },
             # move to drop off
             {'target_pos': deposit_xyz,
              'start_pos': None,
              'orientation': None,
              'n_timesteps': 1000,
              'grasp_command': 'close',
              'hold_timesteps': None,
              'z_offset': 0.01,
              'approach_buffer': 0,
              'traj_planner': second_order_path_planner
             },
             # drop off object
             {'target_pos': deposit_xyz,
              'start_pos': None,
              'orientation': None,
              'n_timesteps': 1000,
              'grasp_command': 'open',
              'hold_timesteps': 500,
              'z_offset': 0.01,
              'approach_buffer': 0,
              'traj_planner': second_order_path_planner
             },
             # lift clear of object in z
             {'target_pos': deposit_xyz,
              'start_pos': None,
              'orientation': None,
              'n_timesteps': 1000,
              'grasp_command': None,
              'hold_timesteps': None,
              'z_offset': 0.35,
              'approach_buffer': 0.0,
              'traj_planner': second_order_path_planner
             }]


    print('\nSimulation starting...\n')

    for reach in reach_list:
        count = 0
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        print('Next reach')
        traj_planner, orientation_planner, target_data = get_approach_path(
            robot_config=robot_config,
            path_planner=reach['traj_planner'](reach['n_timesteps']),
            q=feedback['q'],
            target_pos=reach['target_pos'],
            target_orientation=reach['orientation'],
            start_pos=reach['start_pos'],
            max_reach_dist=None,
            min_z=0.0,
            approach_buffer=reach['approach_buffer'],
            z_offset=reach['z_offset'])
        #
        # while count < reach['n_timesteps']:
        shifted_target = target_data['approach_pos']
        at_target = False
        count = 0
        while not at_target:
            if interface.viewer.exit:
                glfw.destroy_window(interface.viewer.window)
                break

            # get arm feedback
            feedback = interface.get_feedback()
            hand_xyz = robot_config.Tx('EE', feedback['q'])

            pos, vel = traj_planner.next()
            orient = orientation_planner.next()
            target = np.hstack([pos, orient])

            shifted_target += target_shift(interface, scale=0.05)
            interface.set_mocap_xyz('path_planner', shifted_target)
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

            u_gripper = gripper_forces(reach['grasp_command'])

            u = np.hstack((u, u_gripper))

            # apply the control signal, step the sim forward
            interface.send_forces(u)

            error = np.linalg.norm((hand_xyz-target_data['approach_pos']))
            # track data
            ee_track.append(np.copy(hand_xyz))
            ee_angles_track.append(transformations.euler_from_matrix(
                robot_config.R('EE', feedback['q']), axes='rxyz'))
            target_track.append(np.copy(target[:3]))
            target_angles_track.append(np.copy(target[3:]))
            count += 1
            if count % 500 == 0:
                print('error: ', error)
            if reach['hold_timesteps'] is not None:
                if count >= reach['hold_timesteps']:
                    at_target = True
            else:
                if error < 0.02:
                    at_target = True
                elif count > reach['n_timesteps']*2:
                    at_target = True

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
