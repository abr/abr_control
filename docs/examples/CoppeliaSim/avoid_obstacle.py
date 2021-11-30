"""
Move the UR5 CoppeliaSim arm to a target position while avoiding an obstacle.
The simulation ends after 1500 time steps, and the trajectory
of the end-effector is plotted in 3D.
"""
import numpy as np

from abr_control.arms import ur5 as arm

# from abr_control.arms import jaco2 as arm
from abr_control.controllers import OSC, AvoidObstacles, Damping
from abr_control.interfaces import CoppeliaSim

# initialize our robot config
robot_config = arm.Config()

avoid = AvoidObstacles(robot_config)
# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# instantiate the REACH controller with obstacle avoidance
ctrlr = OSC(
    robot_config,
    kp=200,
    null_controllers=[avoid, damping],
    vmax=[0.5, 0],  # [m/s, rad/s]
    # control (x, y, z) out of [x, y, z, alpha, beta, gamma]
    ctrlr_dof=[True, True, True, False, False, False],
)

# create our CoppeliaSim interface
interface = CoppeliaSim(robot_config, dt=0.005)
interface.connect()

# set up lists for tracking data
ee_track = []
target_track = []
obstacle_track = []

moving_obstacle = True
obstacle_xyz = np.array([0.09596, -0.2661, 0.64204])


try:
    # get visual position of end point of object
    feedback = interface.get_feedback()
    start = robot_config.Tx("EE", q=feedback["q"])

    # make the target offset from that start position
    target_xyz = start + np.array([0.2, -0.2, 0.0])
    interface.set_xyz(name="target", xyz=target_xyz)
    interface.set_xyz(name="obstacle", xyz=obstacle_xyz)

    count = 0.0
    obs_count = 0.0
    print("\nSimulation starting...\n")
    while count < 1500:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()

        target = np.hstack(
            [interface.get_xyz("target"), interface.get_orientation("target")]
        )

        # calculate the control signal
        u = ctrlr.generate(
            q=feedback["q"],
            dq=feedback["dq"],
            target=target,
        )

        # get obstacle position from CoppeliaSim
        obs_x, obs_y, obs_z = interface.get_xyz("obstacle")  # pylint: disable=W0632
        # update avoidance system about obstacle position
        avoid.set_obstacles([[obs_x, obs_y, obs_z, 0.05]])
        if moving_obstacle is True:
            obs_x = 0.125 + 0.25 * np.sin(obs_count)
            obs_count += 0.05
            interface.set_xyz(name="obstacle", xyz=[obs_x, obs_y, obs_z])

        # send forces into CoppeliaSim, step the sim forward
        interface.send_forces(u)

        # calculate end-effector position
        ee_xyz = robot_config.Tx("EE", q=feedback["q"])
        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target[:3]))
        obstacle_track.append(np.copy([obs_x, obs_y, obs_z]))

        count += 1

finally:
    # stop and reset the CoppeliaSim simulation
    interface.disconnect()

    print("Simulation terminated...")

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)
    obstacle_track = np.array(obstacle_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        fig = plt.figure(figsize=(8, 12))
        ax1 = fig.add_subplot(211)
        ax1.set_ylabel("Distance (m)")
        ax1.set_xlabel("Time (ms)")
        ax1.set_title("Distance to target")
        ax1.plot(
            np.sqrt(np.sum((np.array(target_track) - np.array(ee_track)) ** 2, axis=1))
        )

        ax2 = fig.add_subplot(212, projection="3d")
        ax2.set_title("End-Effector Trajectory")
        ax2.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label="ee_xyz")
        ax2.scatter(
            target_track[0, 0],
            target_track[0, 1],
            target_track[0, 2],
            label="target",
            c="g",
        )
        ax2.plot(
            obstacle_track[:, 0],
            obstacle_track[:, 1],
            target_track[:, 2],
            label="obstacle",
            c="r",
        )
        ax2.legend()
        plt.show()
