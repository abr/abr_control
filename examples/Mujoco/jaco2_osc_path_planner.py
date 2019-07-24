"""
Example using the full jaco2 model with the following additions
- textures
- rings between joints
- gripper and actuated fingers
- path planner
"""
import glfw
import sys
import timeit
import numpy as np

import mujoco_py as mjp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from abr_control.controllers import OSC, Damping, RestingConfig, path_planners
from abr_control.interfaces.mujoco import Mujoco
from abr_control.arms.mujoco_config import MujocoConfig

use_wall_clock = False

if len(sys.argv) > 1:
    show_plot = sys.argv[1]
else:
    show_plot = False

# NOTE: the meshes file from MuJoCo_Unity_UR5/xmls needs to be in the
# abr_control/arms/jaco2/meshes folder for this to load properly
model_filename = 'jaco2'

# we need to send target angles for the 6 gripper joints as well
# NOTE that only three of the finger joints are actuated
robot_config = MujocoConfig(model_filename)

dt = 0.005
interface = Mujoco(robot_config, dt=dt)
interface.connect()

ctrlr = OSC(robot_config, kp=64, kv=15,
            ctrlr_dof=[True, True, True, False, False, False])

interface.send_target_angles(robot_config.START_ANGLES)

target_xyz = np.array([-0.2, 0.2, 0.4])
interface.set_mocap_xyz('target', target_xyz)

interface.set_mocap_xyz('hand', np.array([.2, .4, 1]))

# create our path planner
params = {}
if use_wall_clock:
    reaching_scale = 2
    run_time = 2  # wall clock time to run each trajectory for
    params['n_timesteps'] = 250  # time steps each trajectory lasts
    time_elapsed = run_time * reaching_scale
    count = 0
else:
    params['error_scale'] = 10
    params['n_timesteps'] = 300  # time steps each trajectory lasts
    count = np.copy(params['n_timesteps'])
    time_elapsed = 0.0
path_planner = path_planners.BellShaped(**params)


ee_track = []
target_track = []

link_name = 'EE'
try:
    while True:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break

        start = timeit.default_timer()
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        if use_wall_clock:
            # either update target every 1s
            update_target = time_elapsed >= run_time * reaching_scale
        else:
            # or update target when trajectory is done
            update_target = (count == params['n_timesteps'])

        if update_target:
            print('TIMES UP')
            count = 0
            time_elapsed = 0.0
            target_xyz[0] = np.random.uniform(0.2, 0.3) * np.sign(np.random.uniform(-1, 1))
            target_xyz[1] = np.random.uniform(0.2, 0.25) * np.sign(np.random.uniform(-1, 1))
            target_xyz[2] = np.random.uniform(0.45, 0.8)
            # update the position of the target
            interface.set_mocap_xyz('target', target_xyz)
            target_track.append(np.copy(target_xyz))

            pos_path, vel_path = path_planner.generate_path(
                position=hand_xyz, target_pos=target_xyz, plot=False)
            if use_wall_clock:
                pos_path = path_planner.convert_to_time(
                    pregenerated_path=pos_path, time_limit=run_time)
                vel_path = path_planner.convert_to_time(
                    pregenerated_path=vel_path, time_limit=run_time)

        # get next target along trajectory
        if use_wall_clock:
            target = [function(min(time_elapsed, run_time)) for function in pos_path]
            target_vel = [function(min(time_elapsed, run_time)) for function in vel_path]
        else:
            target, target_vel = path_planner.next()

        interface.set_mocap_xyz('path_planner', target)
        # generate an operational space control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=np.hstack((target, np.zeros(3))),
            #target_vel=np.hstack((target_vel, np.zeros(3))),
            ref_frame=link_name
            )

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        if count %500 == 0:
            print('ee_xyz: ', hand_xyz)
            print('target_xyz', target_xyz)

        ee_track.append(np.copy(hand_xyz))
        count += 1
        time_elapsed += timeit.default_timer() - start

finally:
    interface.disconnect()

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    if show_plot:
        fig = plt.figure(figsize=(12, 8))
        a1 = fig.add_subplot(1, 1, 1, projection='3d')
        a1.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2])
        print('TT TOT: ', target_track)
        for target in target_track:
            print(target)
            a1.scatter(target[0], target[1], target[2], color='r', marker='o')
        a1.set_title('End-Effector Position')
        a1.set_xlim([-1, 1])
        a1.set_ylim([-1, 1])
        a1.set_zlim([0, 1])
        a1.set_aspect('equal')
        plt.show()
