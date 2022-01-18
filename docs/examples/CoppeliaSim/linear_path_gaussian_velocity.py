"""
Running operational space control using CoppeliaSim. The controller will
move the end-effector to the target object's position and orientation.

This example controls all 6 degrees of freedom (position and orientation),
and applies a second order path planner to both position and orientation targets

After termination the script will plot results
"""
import numpy as np

from abr_control.arms import ur5 as arm
from abr_control.controllers import OSC, Damping
from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.velocity_profiles import Gaussian
from abr_control.interfaces import CoppeliaSim
from abr_control.utils import transformations

dt = 0.005
# initialize our robot config
robot_config = arm.Config()

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create opreational space controller
ctrlr = OSC(
    robot_config,
    kp=100,  # position gain
    ko=250,  # orientation gain
    null_controllers=[damping],
    vmax=None,  # [m/s, rad/s]
    # control all DOF [x, y, z, alpha, beta, gamma]
    ctrlr_dof=[True, True, True, True, True, True],
)

# create our interface
interface = CoppeliaSim(robot_config, dt=dt)
interface.connect()

# pregenerate our path and orientation planners
n_timesteps = 100

path_planner = PathPlanner(
        pos_profile=Linear(),
        vel_profile=Gaussian(dt=dt, acceleration=2)
)

# traj_planner = path_planners.SecondOrderDMP(error_scale=50, n_timesteps=n_timesteps)
# orientation_planner = path_planners.Orientation()

feedback = interface.get_feedback()
hand_xyz = robot_config.Tx("EE", feedback["q"])
starting_orientation = robot_config.quaternion("EE", feedback["q"])

target_orientation = np.random.random(3)
target_orientation /= np.linalg.norm(target_orientation)
# convert our orientation to a quaternion
target_orientation = [0] + list(target_orientation)
target_position = [-0.4, -0.3, 0.6]

starting_orientation = transformations.euler_from_quaternion(
        starting_orientation, axes='rxyz')

target_orientation = transformations.euler_from_quaternion(
        target_orientation, axes='rxyz')

path_planner.generate_path(
        start_position=hand_xyz, target_position=target_position,
        start_orientation=starting_orientation, target_orientation=target_orientation,
        max_velocity=2
)

# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []


try:
    count = 0
    interface.set_xyz("target", target_position)
    interface.set_orientation(
        "target", target_orientation
    )

    print("\nSimulation starting...\n")
    while count < path_planner.n_timesteps:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx("EE", feedback["q"])

        next_target = path_planner.next()
        pos = next_target[:3]
        vel = next_target[3:6]
        orient = next_target[6:9]

        target = np.hstack([pos, orient])

        u = ctrlr.generate(
            q=feedback["q"],
            dq=feedback["dq"],
            target=target,
            target_velocity=np.hstack([vel, np.zeros(3)])
        )

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
