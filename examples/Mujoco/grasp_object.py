"""
Running operational space control using Mujoco. The controller will
move the end-effector to the target object's position and orientation.

This example controls all 6 degrees of freedom (position and orientation),
and applies a second order path planner to both position and orientation targets

After termination the script will plot results
"""
import numpy as np
import glfw
import time

from abr_control.controllers import OSC, Damping, path_planners
from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.interfaces.mujoco import Mujoco
from abr_control.utils import transformations


# initialize our robot config
robot_config = arm('jaco2_gripper')

# damp the movements of the arm
damping = Damping(robot_config, kv=10)

# create our interface
interface = Mujoco(robot_config, dt=.001)
interface.connect()

feedback = interface.get_feedback()
hand_xyz = robot_config.Tx('EE', feedback['q'])

def gripper_forces(command=None, grip_force=3.0):
    """
    accepts open or close and returns a gripper force in the corresponding direction

    Parameters
    ----------
    command: string
        open to open gripper, close to close it
    grip_force: float, optional (Default: 3.0)
        the gripper force in Nm
    """
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

def osc6dof():
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
    return ctrlr

def osc3dof():
    # create opreational space controller
    ctrlr = OSC(
        robot_config,
        kp=30,  # position gain
        kv=20,
        null_controllers=[damping],
        vmax=None,  # [m/s, rad/s]
        # control all DOF [x, y, z, alpha, beta, gamma]
        ctrlr_dof = [True, True, True, False, False, False])
    return ctrlr



def second_order_path_planner(n_timesteps=2000, error_scale=0.25):
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

def target_shift(interface, base_location, scale=0.01, xlim=None, ylim=None, zlim=None):
    """
    Gets the user input from the mujoco_viewer to shift the target and returns the
    base_location with the shift*scale. The final target is clipped along x, y, z
    depending on the xlim, ylim, and zlim values

    Parameters
    ----------
    interface: abr_control.interface.mujoco_interface class
    base_location: array of 3 floats
        the current target to be shifted [meters]
    scale: float, optional (Default: 0.01)
        the amount to move with each button press [meters]
    xlim: list of 2 floats, Optional (Default: -1, 1)
        the minimum and maximum allowable values [meters]
    ylim: list of 2 floats, Optional (Default: -1, 1)
        the minimum and maximum allowable values [meters]
    zlim: list of 2 floats, Optional (Default: 0, 1)
        the minimum and maximum allowable values [meters]
    """
    if xlim is None:
        xlim = [-1, 1]
    if ylim is None:
        ylim = [-1, 1]
    if zlim is None:
        zlim = [0, 1]

    def clip(val, minimum, maximum):
        val = max(val, minimum)
        val = min(val, maximum)
        return val

    shifted_target = base_location + scale * np.array([
        interface.viewer.target_x,
        interface.viewer.target_y,
        interface.viewer.target_z])
    shifted_target = np.array([
        clip(shifted_target[0], xlim[0], xlim[1]),
        clip(shifted_target[1], ylim[0], ylim[1]),
        clip(shifted_target[2], zlim[0], zlim[1])])
    interface.viewer.target_x = 0
    interface.viewer.target_y = 0
    interface.viewer.target_z = 0

    return shifted_target

# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []
object_xyz = np.array([0, 0.5, 0.3])
deposit_xyz = np.array([0.4, 0.5, 0.0])


try:
    reaching_list = [
        # GRASP AND LIFT
        # move above object
        {'type': 'grasp',
        'target_pos': object_xyz,
        'start_pos': None,
        'orientation': None,
        'n_timesteps': 2000,
        'grasp_command': 'open',
        'hold_timesteps': None,
        'z_offset': 0.2,
        'approach_buffer': 0.02,
        'ctrlr': osc6dof,
        'traj_planner': second_order_path_planner
        },
        # get into grasping position
        {'type': 'grasp',
        'target_pos': object_xyz,
        'start_pos': None,
        'orientation': None,
        'n_timesteps': 1000,
        'grasp_command': 'open',
        'hold_timesteps': None,
        'z_offset': 0,
        'approach_buffer': 0,
        'ctrlr': osc6dof,
        'traj_planner': second_order_path_planner
        },
        # grasp object
        {'type': 'grasp',
        'target_pos': object_xyz,
        'start_pos': None,
        'orientation': None,
        'n_timesteps': 1000,
        'grasp_command': 'close',
        'hold_timesteps': 500,
        'z_offset': 0,
        'approach_buffer': 0,
        'ctrlr': osc6dof,
        'traj_planner': second_order_path_planner
        },
        # lift object
        {'type': 'grasp',
        'target_pos': object_xyz,
        'start_pos': None,
        'orientation': None,
        'n_timesteps': 1000,
        'grasp_command': 'close',
        'hold_timesteps': None,
        'z_offset': 0.2,
        'approach_buffer': 0.03,
        'ctrlr': osc6dof,
        'traj_planner': second_order_path_planner
        },

        # MOVE TO TARGET AND DROP OFF
        # move above drop off
        # {'type': 'target_reach',
        # 'target_pos': deposit_xyz,
        # 'start_pos': None,
        # 'orientation': None,
        # 'n_timesteps': 1000,
        # 'grasp_command': 'close',
        # 'hold_timesteps': None,
        # 'z_offset': 0.5,
        # 'approach_buffer': 0.0,
        # 'traj_planner': second_order_path_planner
        # },
        # move to drop off
        {'type': 'target_reach',
        'target_pos': deposit_xyz,
        'start_pos': None,
        'orientation': None,
        'n_timesteps': 1000,
        'grasp_command': 'close',
        'hold_timesteps': None,
        'ctrlr': osc3dof,
        'z_offset': 0.35,
        'approach_buffer': 0,
        'traj_planner': second_order_path_planner
        },
        # drop off object
        # {'type': 'target_reach',
        # 'target_pos': deposit_xyz,
        # 'start_pos': None,
        # 'orientation': None,
        # 'n_timesteps': 1000,
        # 'grasp_command': 'open',
        # 'hold_timesteps': 500,
        # 'z_offset': 0.31,
        # 'approach_buffer': 0,
        # 'traj_planner': second_order_path_planner
        # },
        # lift clear of object in z
        # {'type': 'target_reach',
        # 'target_pos': deposit_xyz,
        # 'start_pos': None,
        # 'orientation': None,
        # 'n_timesteps': 1000,
        # 'grasp_command': None,
        # 'hold_timesteps': None,
        # 'z_offset': 0.65,
        # 'approach_buffer': 0.0,
        # 'traj_planner': second_order_path_planner
        # }
        ]


    print('\nSimulation starting...\n')

    final_xyz = deposit_xyz

    # this can later be expaned to actually check the user input to start reaching
    # for object to grasp, or for different objects, for now just set it to True
    interface.viewer.pick_up_object = True

    # wait until the user hits the 'pick up object' button
    while not interface.viewer.pick_up_object:
        time.sleep(0.5)

    for reach in reaching_list:
        count = 0
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        if reach['type'] == 'target_reach':
            reach['target_pos'] = final_xyz

        print('Next reach')
        # calculate our position and orientation path planners, with their
        # corresponding approach
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

        at_target = False
        count = 0
        while not at_target:
            # check for our exit command (caps lock)
            if interface.viewer.exit:
                glfw.destroy_window(interface.viewer.window)
                break

            # get our user shifted drop off location
            old_final_xyz = final_xyz
            final_xyz = target_shift(
                interface=interface,
                base_location=final_xyz,
                scale=0.05,
                xlim=[-0.5, 0.5],
                ylim=[-0.5, 0.5],
                zlim=[0.0, 0.7])

            # get arm feedback
            feedback = interface.get_feedback()
            hand_xyz = robot_config.Tx('EE', feedback['q'])

            # update our path planner position and orientation
            if reach['type'] == 'target_reach':
                error = np.linalg.norm(
                    (hand_xyz - (final_xyz + np.array([0, 0, reach['z_offset']]))))
                if not np.allclose(final_xyz, old_final_xyz, atol=1e-5):
                    traj_planner.reset(
                        position=hand_xyz,
                        target_pos=(final_xyz + np.array([0, 0, reach['z_offset']])))
                    print('RESETTING DMP')
                pos, vel = traj_planner._step(error=error)

            else:
                error = np.linalg.norm((hand_xyz-target_data['approach_pos']))
                pos, vel = traj_planner.next()

            #TODO will need to update the orientation planner somehow, not using
            # dmps so can't use reset, may need to regen and change n_timesteps?
            orient = orientation_planner.next()
            target = np.hstack([pos, orient])

            # set our path planner visualization and final drop off location
            interface.set_mocap_xyz('target', final_xyz)
            interface.set_mocap_xyz('path_planner_orientation', target[:3])
            interface.set_mocap_orientation('path_planner_orientation',
                transformations.quaternion_from_euler(
                    orient[0], orient[1], orient[2], 'rxyz'))

            # calculate our osc control signal
            u = reach['ctrlr']().generate(
                q=feedback['q'],
                dq=feedback['dq'],
                target=target,
                #target_vel=np.hstack([vel, np.zeros(3)])
                )

            # get our gripper command
            u_gripper = gripper_forces(reach['grasp_command'])

            # stack our control signals and send to mujoco, stepping the sim forward
            u = np.hstack((u, u_gripper))
            interface.send_forces(u)

            # calculate our 2norm error
            if count % 500 == 0:
                print('error: ', error)

            # track data
            ee_track.append(np.copy(hand_xyz))
            ee_angles_track.append(transformations.euler_from_matrix(
                robot_config.R('EE', feedback['q']), axes='rxyz'))
            target_track.append(np.copy(target[:3]))
            target_angles_track.append(np.copy(target[3:]))
            count += 1

            # once we have the object, keep reaching to the target as the user
            # changes it
            if reach['type'] == 'target_reach':
                at_target = False
            # the reason we differentiate hold and n timesteps is that hold is how
            # long we want to wait to allow for the action, mainly used for grasping,
            # whereas n_timesteps determines the number of steps in the path planner.
            # we check n_timesteps*2 to allow the arm to catch up to the path planner
            else:
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
