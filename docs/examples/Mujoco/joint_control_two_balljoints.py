"""
Move the jao2 Mujoco arm to a target position.
The simulation ends after 1500 time steps, and the
trajectory of the end-effector is plotted in 3D.
"""
import traceback
import numpy as np
import glfw

import mujoco_py as mjp

from abr_control.controllers import Joint
from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.interfaces.mujoco import Mujoco
from abr_control.utils import transformations
from abr_control.utils.transformations import quaternion_multiply, quaternion_conjugate


# initialize our robot config for the jaco2
robot_config = arm("mujoco_two_balljoints.xml", folder=".", use_sim_state=False)

# create our Mujoco interface
interface = Mujoco(robot_config, dt=0.001)
interface.connect()

# instantiate controller
kp = 100
kv = np.sqrt(kp)
ctrlr = Joint(
    robot_config,
    kp=kp,
    kv=kv,
    quaternions=[True, True],
)

# set up lists for tracking data
q_track = []
target_track = []
error_track = []

target_geom_id = interface.sim.model.geom_name2id("target")
green = [0, 0.9, 0, 0.5]
red = [0.9, 0, 0, 0.5]
threshold = 0.002  # threshold distance for being within target before moving on

# get the end-effector's initial position
np.random.seed(0)
target_quaternions = [np.hstack([transformations.unit_vector(
    transformations.random_quaternion()) for jj in range(2)]) for ii in range(4)]

target_index = 0
target = target_quaternions[target_index]
print(target)
target_xyz = robot_config.Tx('EE', q=target)
interface.set_mocap_xyz(name="target", xyz=target_xyz)

try:
    # get the end-effector's initial position
    feedback = interface.get_feedback()
    start = robot_config.Tx("EE", feedback["q"])

    count = 0.0
    print("\nSimulation starting...\n")
    while 1:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()

        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback["dq"],
            target=target,
        )

        # send forces into Mujoco, step the sim forward
        interface.send_forces(u)

        # track data
        q_track.append(np.copy(feedback['q']))
        target_track.append(np.copy(target))

        # calculate the distance between quaternions
        error = 0.0
        for ii in range(2):
            temp = quaternion_multiply(
                target[ii*4:(ii*4)+4],
                quaternion_conjugate(feedback['q'][ii*4:(ii*4)+4]),
            )
            temp = 2 * np.arctan2(np.linalg.norm(temp[1:]) * -np.sign(temp[0]), temp[0])
            # quaternion distance for same angles can be 0 or 2*pi, so use a sine
            # wave here so 0 and 2*np.pi = 0
            error += np.sin(temp / 2)
        error_track.append(np.copy(error))

        if abs(error) < threshold:
            interface.sim.model.geom_rgba[target_geom_id] = green
            count += 1
        else:
            count = 0
            interface.sim.model.geom_rgba[target_geom_id] = red

        if count >= 1000:
            target_index += 1
            target = target_quaternions[target_index % len(target_quaternions)]
            target_xyz = robot_config.Tx('EE', q=target)
            interface.set_mocap_xyz(name="target", xyz=target_xyz)
            count = 0

except:
    print(traceback.format_exc())

finally:
    # stop and reset the Mujoco simulation
    interface.disconnect()

    print("Simulation terminated...")

    q_track = np.array(q_track)
    target_track = np.array(target_track)

    if q_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        plt.figure(figsize=(8, 6))
        plt.ylabel("Distance (m)")
        plt.xlabel("Time (ms)")
        plt.title("Distance to target")
        plt.plot(np.array(error_track))
        plt.show()
