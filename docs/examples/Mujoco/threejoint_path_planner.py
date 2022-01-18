"""
Example using the mujoco interface with a three link arm
reaching with a path planner

To plot the joint angles call the script with True
passed through sys.argv
    python threelink.py True
"""

import sys
import timeit

import glfw
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # pylint: disable=W0611

from abr_control.arms.mujoco_config import MujocoConfig
from abr_control.controllers import OSC
from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.velocity_profiles import Gaussian
from abr_control.interfaces.mujoco import Mujoco

if len(sys.argv) > 1:
    use_wall_clock = True
else:
    use_wall_clock = False
    print(
        "To plan the path based on real time instead of steps, append"
        + " 'True' to your script call"
    )


if len(sys.argv) > 1:
    show_plot = sys.argv[1]
else:
    show_plot = False
model_filename = "threejoint"

robot_config = MujocoConfig(model_filename)

dt = 0.001
interface = Mujoco(robot_config, dt=dt)
interface.connect()
interface.send_target_angles(robot_config.START_ANGLES)

ctrlr = OSC(
    robot_config, kp=30, kv=20, ctrlr_dof=[True, True, True, False, False, False]
)

interface.send_target_angles(np.ones(3))

target_xyz = np.array([0.1, 0.1, 0.3])
interface.set_mocap_xyz("target", target_xyz)

interface.set_mocap_xyz("hand", np.array([0.2, 0.4, 1]))

target_geom_id = interface.sim.model.geom_name2id("target")
green = [0, 0.9, 0, 0.5]
red = [0.9, 0, 0, 0.5]

# create our path planner
params = {}
if use_wall_clock:
    run_time = 4  # wall clock time to run each trajectory for
    params["n_timesteps"] = 100  # time steps each trajectory lasts
    time_elapsed = np.copy(run_time)
    count = 0
else:
    params["n_timesteps"] = 2000  # time steps each trajectory lasts
    count = np.copy(params["n_timesteps"])
    time_elapsed = 0.0
path_planner = PathPlanner(
    pos_profile=Linear(),
    vel_profile=Gaussian(dt=dt, acceleration=1)
)

ee_track = []
target_track = []

count = 0
update_target = True
link_name = "EE"
try:
    while True:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break

        start = timeit.default_timer()
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx("EE", feedback["q"])

        if update_target:
            count = 0
            time_elapsed = 0.0
            target_xyz[0] = np.random.uniform(0.2, 0.3) * np.sign(
                np.random.uniform(-1, 1)
            )
            target_xyz[1] = np.random.uniform(0.2, 0.25) * np.sign(
                np.random.uniform(-1, 1)
            )
            target_xyz[2] = np.random.uniform(0.3, 0.4)
            # update the position of the target
            interface.set_mocap_xyz("target", target_xyz)

            generated_path = path_planner.generate_path(
                start_position=hand_xyz,
                target_position=target_xyz,
                max_velocity=1,
                plot=False
            )
            pos_path = generated_path[:, :3]
            vel_path = generated_path[:, 3:6]

            if use_wall_clock:
                pos_path = path_planner.convert_to_time(
                    path=pos_path, time_length=run_time
                )
                vel_path = path_planner.convert_to_time(
                    path=vel_path, time_length=run_time
                )

        # get next target along trajectory
        if use_wall_clock:
            target = [function(min(time_elapsed, run_time)) for function in pos_path]
            target_velocity = [
                function(min(time_elapsed, run_time)) for function in vel_path
            ]
        else:
            next_target = path_planner.next()
            target = next_target[:3]
            target_velocity = next_target[3:]

        interface.set_mocap_xyz("path_planner", target)
        # generate an operational space control signal
        u = ctrlr.generate(
            q=feedback["q"],
            dq=feedback["dq"],
            target=np.hstack((target, np.zeros(3))),
            ref_frame=link_name,
        )

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        ee_track.append(np.copy(hand_xyz))
        target_track.append(interface.get_xyz(link_name))
        count += 1
        time_elapsed += timeit.default_timer() - start

        error = np.linalg.norm(hand_xyz - target_xyz)
        if error < 0.02:
            interface.sim.model.geom_rgba[target_geom_id] = green
        else:
            interface.sim.model.geom_rgba[target_geom_id] = red

        if count % 500 == 0:
            print("error: ", error)

        if use_wall_clock:
            # either update target every 1s
            update_target = time_elapsed >= path_planner.time_to_converge + 2
        else:
            # or update target when trajectory is done
            update_target = count == path_planner.n_timesteps + 500


finally:
    interface.disconnect()

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    if show_plot:
        plt.figure()

        plt.subplot(1, 2, 1, projection="3d")
        plt.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2])
        plt.plot(
            target_track[:, 0],
            target_track[:, 1],
            target_track[:, 2],
            "rx",
            mew=3,
            lw=2,
        )
        plt.gca().set_xlim([-1, 1])
        plt.gca().set_ylim([-1, 1])
        plt.gca().set_zlim([0, 1])
        plt.gca().set_aspect("equal")

        plt.subplot(1, 2, 2)
        plt.plot(ee_track, lw=2)
        plt.gca().set_prop_cycle(None)
        plt.plot(target_track, "--", lw=2)

        plt.show()
