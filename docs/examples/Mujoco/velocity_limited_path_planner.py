"""
Running operational space control using Mujoco. The controller will
move the end-effector to the target object's X position and orientation.

The cartesian direction being controlled is set in the first three booleans
of the ctrlr_dof parameter
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

max_a = 2
n_targets = 100

if len(sys.argv) > 1:
    arm_model = sys.argv[1]
else:
    arm_model = "jaco2"
# initialize our robot config for the jaco2
robot_config = arm(arm_model)

ctrlr_dof = [True, True, True, False, False, False]
dof_labels = ["x", "y", "z", "a", "b", "g"]
dof_print = f"* DOF Controlled: {np.array(dof_labels)[ctrlr_dof]} *"
stars = "*" * len(dof_print)
print(stars)
print(dof_print)
print(stars)
dt = 0.001

# create our interface
interface = Mujoco(robot_config, dt=dt)
interface.connect()
interface.send_target_angles(robot_config.START_ANGLES)

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
    ctrlr_dof=ctrlr_dof,
)

path_planner = PathPlanner(
        pos_profile=Linear(),
        vel_profile=Gaussian(dt=dt, acceleration=max_a)
)

# set up lists for tracking data
ee_track = []
target_track = []

try:
    print("\nSimulation starting...\n")
    for ii in range(0, n_targets):
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx("EE", feedback["q"])

        pos_target = np.array([
                np.random.uniform(low=-0.4, high=0.4),
                np.random.uniform(low=-0.4, high=0.4),
                np.random.uniform(low=0.3, high=0.6)])

        # ang_target = np.array([
        #         0, 0,
        #         np.random.uniform(low=0, high=2*np.pi)])

        # starting_orientation = robot_config.quaternion("EE", feedback["q"])
        # ang_state = transformations.euler_from_quaternion(starting_orientation, 'rxyz')

        path_planner.generate_path(
                start_position=hand_xyz,
                target_position=pos_target,
                max_velocity=2
                )

        interface.set_mocap_xyz("target", pos_target)
        at_target = 0
        count = 0

        while at_target < 500:
            if count > 5000:
                break
            filtered_target = path_planner.next()
            interface.set_mocap_xyz("target_orientation", filtered_target[:3])
            # interface.set_mocap_orientation(
            #         "target_orientation",
            #         transformations.quaternion_from_euler(
            #             filtered_target[6],
            #             filtered_target[7],
            #             filtered_target[8],
            #             'rxyz'))

            feedback = interface.get_feedback()
            hand_xyz = robot_config.Tx("EE", feedback["q"])

            if interface.viewer.exit:
                glfw.destroy_window(interface.viewer.window)
                break

            u = ctrlr.generate(
                q=feedback["q"],
                dq=feedback["dq"],
                target=filtered_target,
            )

            # add gripper forces
            u = np.hstack((u, np.zeros(robot_config.N_GRIPPER_JOINTS)))

            # apply the control signal, step the sim forward
            interface.send_forces(u)

            # track data
            ee_track.append(np.copy(hand_xyz))
            target_track.append(np.copy(pos_target[:3]))

            if np.linalg.norm(hand_xyz-pos_target) < 0.02:
                at_target += 1
            else:
                at_target = 0
            count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print("Simulation terminated...")

    ee_track = np.array(ee_track)
    # ee_angles_track = np.array(ee_angles_track)
    target_track = np.array(target_track)
    # target_angles_track = np.array(target_angles_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        fig = plt.figure(figsize=(8, 12))
        ax1 = fig.add_subplot(311)
        ax1.set_ylabel("3D position (m)")
        for ii, controlled_dof in enumerate(ctrlr_dof[:3]):
            if controlled_dof:
                ax1.plot(ee_track[:, ii], label=dof_labels[ii])
                ax1.plot(target_track[:, ii], "--")
        ax1.legend()

        # ax2 = fig.add_subplot(312)
        # for ii, controlled_dof in enumerate(ctrlr_dof[3:]):
        #     if controlled_dof:
        #         ax2.plot(ee_angles_track[:, ii], label=dof_labels[ii + 3])
        #         ax2.plot(target_angles_track[:, ii], "--")
        # ax2.set_ylabel("3D orientation (rad)")
        # ax2.set_xlabel("Time (s)")
        # ax2.legend()

        ax3 = fig.add_subplot(313, projection="3d")
        ax3.set_title("End-Effector Trajectory")
        ax3.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label="ee_xyz")
        ax3.scatter(
            target_track[0, 0],
            target_track[0, 1],
            target_track[0, 2],
            label="target",
            c="g",
        )
        ax3.legend()
        plt.show()
