"""
Move the jaco2 Mujoco arm to a target position.
The simulation ends after 1.5 simulated seconds, and the
trajectory of the end-effector is plotted in 3D.
"""
import sys
import traceback

import glfw
import numpy as np

from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.controllers import OSC, Damping, signals
from abr_control.interfaces.mujoco import Mujoco
from abr_control.utils import transformations

if len(sys.argv) > 1:
    arm_model = sys.argv[1]
else:
    arm_model = "jaco2"
# initialize our robot config for the jaco2
robot_config = arm(arm_model)

# create our Mujoco interface
interface = Mujoco(robot_config, dt=0.001)
interface.connect()
interface.send_target_angles(robot_config.START_ANGLES)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# instantiate controller
ctrlr = OSC(
    robot_config,
    kp=200,
    null_controllers=[damping],
    vmax=[0.5, 0],  # [m/s, rad/s]
    # control (x, y, z) out of [x, y, z, alpha, beta, gamma]
    ctrlr_dof=[True, True, True, False, False, False],
)

# create our adaptive controller
adapt = signals.DynamicsAdaptation(
    n_neurons=5000,
    n_ensembles=5,
    n_input=10,  # we apply adaptation on the most heavily stressed joints
    n_output=5,
    pes_learning_rate=1e-4,
    means=[0.12, 2.14, 1.87, 4.32, 0.59, 0.12, -0.38, -0.42, -0.29, 0.36],
    variances=[0.08, 0.6, 0.7, 0.3, 0.6, 0.08, 1.4, 1.6, 0.7, 1.2],
    spherical=True,
)

target_geom_id = interface.sim.model.geom_name2id("target")
green = [0, 0.9, 0, 0.5]
red = [0.9, 0, 0, 0.5]

# set up lists for tracking data
ee_track = []
target_track = []
q_track = []
dq_track = []

try:
    # get the end-effector's initial position
    feedback = interface.get_feedback()
    start = robot_config.Tx("EE", feedback["q"])

    # make the target offset from that start position
    target_xyz = start + np.array([0.2, -0.2, -0.2])
    interface.set_mocap_xyz(name="target", xyz=target_xyz)
    # set the status of the top right text for adaptation
    interface.viewer.adapt = True

    count = 0.0
    while 1:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break

        # get joint angle and velocity feedback
        feedback = interface.get_feedback()

        target = np.hstack(
            [
                interface.get_xyz("target"),
                transformations.euler_from_quaternion(
                    interface.get_orientation("target"), "rxyz"
                ),
            ]
        )

        # calculate the control signal
        u = ctrlr.generate(
            q=feedback["q"],
            dq=feedback["dq"],
            target=target,
        )

        u_adapt = np.zeros(robot_config.N_JOINTS)
        u_adapt[:5] = adapt.generate(
            input_signal=np.hstack((feedback["q"][:5], feedback["dq"][:5])),
            training_signal=np.array(ctrlr.training_signal[:5]),
        )
        u += u_adapt

        # add an additional force for the controller to adapt to
        extra_gravity = robot_config.g(feedback["q"]) * 4
        u += extra_gravity

        # add gripper forces
        u = np.hstack((u, np.zeros(robot_config.N_GRIPPER_JOINTS)))

        # send forces into Mujoco, step the sim forward
        interface.send_forces(u)

        # calculate end-effector position
        ee_xyz = robot_config.Tx("EE", q=feedback["q"])
        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target[:3]))
        q_track.append(np.copy(feedback["q"]))
        dq_track.append(np.copy(feedback["dq"]))

        count += 1

        error = np.linalg.norm(ee_xyz - target[:3])
        if error < 0.02:
            interface.sim.model.geom_rgba[target_geom_id] = green
        else:
            interface.sim.model.geom_rgba[target_geom_id] = red

except:
    print(traceback.format_exc())

finally:
    # stop and reset the Mujoco simulation
    interface.disconnect()

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)
    q_track = np.asarray(q_track)
    dq_track = np.asarray(dq_track)

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
        ax2.scatter(target_xyz[0], target_xyz[1], target_xyz[2], label="target", c="r")
        ax2.legend()
        plt.show()
