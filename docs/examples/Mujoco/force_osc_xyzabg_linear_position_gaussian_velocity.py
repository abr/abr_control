"""
Running operational space control using Mujoco. The controller will
move the end-effector to the target object's position and orientation.

This example controls all 6 degrees of freedom (position and orientation),
and applies a second order path planner to both position and orientation targets

After termination the script will plot results
"""
import sys

import glfw
import numpy as np

from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.controllers import OSC, Damping
from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.velocity_profiles import Gaussian
from abr_control.interfaces.mujoco import Mujoco
from abr_control.utils import transformations

dt = 0.001
# initialize our robot config
if len(sys.argv) > 1:
    arm_model = sys.argv[1]
else:
    arm_model = "jaco2"
# initialize our robot config for the jaco2
robot_config = arm(arm_model)

# create our interface
interface = Mujoco(robot_config, dt=dt)
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
    ctrlr_dof=[True, True, True, True, True, True],
)

feedback = interface.get_feedback()
hand_xyz = robot_config.Tx("EE", feedback["q"])

path_planner = PathPlanner(
    pos_profile=Linear(), vel_profile=Gaussian(dt=dt, acceleration=2)
)


# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []
first_pass = True


try:
    count = 0

    print("\nSimulation starting...\n")
    while 1:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx("EE", feedback["q"])
        if first_pass or count == path_planner.n_timesteps + 500:
            count = 0
            first_pass = False

            # pregenerate our path and orientation planners
            q = robot_config.quaternion("EE", feedback["q"])
            starting_orientation = transformations.euler_from_quaternion(q, axes="rxyz")

            mag = 0.6
            target_position = np.random.random(3) * 0.5
            target_position = target_position / np.linalg.norm(target_position) * mag

            target_orientation = np.random.uniform(low=-np.pi, high=np.pi, size=3)

            path_planner.generate_path(
                start_position=hand_xyz,
                target_position=target_position,
                start_orientation=starting_orientation,
                target_orientation=target_orientation,
                max_velocity=2,
            )

            interface.set_mocap_xyz("target_orientation", target_position)
            interface.set_mocap_orientation(
                "target_orientation",
                transformations.quaternion_from_euler(
                    target_orientation[0],
                    target_orientation[1],
                    target_orientation[2],
                    "rxyz",
                ),
            )

        next_target = path_planner.next()
        pos = next_target[:3]
        vel = next_target[3:6]
        orient = next_target[6:9]
        target = np.hstack([pos, orient])

        interface.set_mocap_xyz("path_planner_orientation", target[:3])
        interface.set_mocap_orientation(
            "path_planner_orientation",
            transformations.quaternion_from_euler(
                orient[0], orient[1], orient[2], "rxyz"
            ),
        )

        u = ctrlr.generate(
            q=feedback["q"],
            dq=feedback["dq"],
            target=target,
        )

        # add gripper forces
        u = np.hstack((u, np.zeros(robot_config.N_GRIPPER_JOINTS)))

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # track data
        ee_track.append(np.copy(hand_xyz))
        ee_angles_track.append(
            transformations.euler_from_matrix(
                robot_config.R("EE", feedback["q"]), axes="rxyz"
            )
        )
        target_track.append(np.copy(target[:3]))
        target_angles_track.append(np.copy(target[3:]))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print("Simulation terminated...")

    ee_track = np.array(ee_track).T
    ee_angles_track = np.array(ee_angles_track).T
    target_track = np.array(target_track).T
    target_angles_track = np.array(target_angles_track).T

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        label_pos = ["x", "y", "z"]
        label_or = ["a", "b", "g"]
        c = ["r", "g", "b"]

        fig = plt.figure(figsize=(8, 12))
        ax1 = fig.add_subplot(311)
        ax1.set_ylabel("3D position (m)")
        for ii, ee in enumerate(ee_track):
            ax1.plot(ee, label=f"EE: {label_pos[ii]}", c=c[ii])
            ax1.plot(
                target_track[ii],
                label=f"Target: {label_pos[ii]}",
                c=c[ii],
                linestyle="--",
            )
        ax1.legend()

        ax2 = fig.add_subplot(312)
        for ii, ee in enumerate(ee_angles_track):
            ax2.plot(ee, label=f"EE: {label_or[ii]}", c=c[ii])
            ax2.plot(
                target_angles_track[ii],
                label=f"Target: {label_or[ii]}",
                c=c[ii],
                linestyle="--",
            )
        ax2.set_ylabel("3D orientation (rad)")
        ax2.set_xlabel("Time (s)")
        ax2.legend()

        ee_track = ee_track.T
        target_track = target_track.T
        ax3 = fig.add_subplot(313, projection="3d")
        ax3.set_title("End-Effector Trajectory")
        ax3.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label="ee_xyz")
        ax3.plot(
            target_track[:, 0],
            target_track[:, 1],
            target_track[:, 2],
            label="ee_xyz",
            c="g",
            linestyle="--",
        )
        ax3.scatter(
            target_track[-1, 0],
            target_track[-1, 1],
            target_track[-1, 2],
            label="target",
            c="g",
        )
        ax3.legend()
        plt.show()
