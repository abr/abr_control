"""
Example of rover driving to red ball targets using vision feedback

Simplified version of example in
https://github.com/abr/neurorobotics-2020
"""
import traceback
import sys
import glfw
import timeit
import os

import matplotlib.pyplot as plt
import numpy as np

from abr_control.arms.mujoco_config import MujocoConfig
from abr_control.interfaces.mujoco import Mujoco

class ExitSim(Exception):
    pass

current_dir = os.path.abspath(".")
if not os.path.exists("%s/figures" % current_dir):
    os.makedirs("%s/figures" % current_dir)
if not os.path.exists("%s/data" % current_dir):
    os.makedirs("%s/data" % current_dir)

# plot the camera feedback every 500 steps
camera_vis_fps = 500
n_targets = 2
seed = 0

np.random.seed(seed)
# target generation limits
dist_limit = [0.25, 3.5]
angle_limit = [-np.pi, np.pi]

accel_scale = 500
steer_scale = 500

# data collection parameters in steps (1ms per step)
max_time_to_target = 10000

rover = MujocoConfig("rover.xml", folder='')
dt = 0.001

# create our Mujoco interface
interface = Mujoco(
    robot_config=rover,
    dt=dt,
    offscreen_render_params={
        "cameras": [4, 1, 3, 2],  # camera ids and order to render
        "resolution": [32, 32],
        "frequency": 1,  # render images from cameras every time step
    },
)
interface.connect(joint_names=["steering_wheel", "RR_joint"])

# set up the target position
target = np.array([-0.0, 0.0, 0.2])

# track position
target_track = []
rover_track = []
vision_track = []
localtarget_track = []
ideal_motor_track = []

# rotation matrix for rotating error from world to rover frame
theta = 3 * np.pi / 2
R90 = np.array(
    [
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1],
    ]
)

total_time = 0
step = 0
try:
    while True:
        if glfw.window_should_close(interface.viewer.window):
            break

        rover_xyz = interface.get_xyz("base_link")
        dist = np.linalg.norm(rover_xyz - target)
        if dist < 0.2 or int(total_time / interface.dt) % max_time_to_target == 0:
            # generate a new target 1-2.5m away from current position
            while dist < 1 or dist > 2.5:
                phi = np.random.uniform(low=angle_limit[0], high=angle_limit[1])
                radius = np.random.uniform(low=dist_limit[0], high=dist_limit[1])
                target = [np.cos(phi) * radius, np.sin(phi) * radius, 0.2]
                dist = np.linalg.norm(rover_xyz - target)
            interface.set_mocap_xyz("target", target)
            target_track.append(target)

        # track data
        rover_track.append(rover_xyz)

        if len(target_track) == n_targets:
            break

        rover_xyz = interface.get_xyz("base_link")
        error = target - rover_xyz
        # error in global coordinates, want it in local coordinates for rover
        quaternion = interface.get_orientation("base_link")#.T  # R.T = R^-1
        w, x, y, z = quaternion
        R_raw = np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
        ]).T

        # rotate it so y points forward toward the steering wheels
        R = np.dot(R90, R_raw)
        local_target = np.dot(R, error)
        localtarget_track.append(local_target / np.pi)

        # used to roughly normalize the local target signal to -1:1 range
        local_err = np.array([local_target[0], local_target[1]])

        feedback = interface.get_feedback()
        # steer_error = np.array([feedback['q'][0], local_err[1]]) * np.pi  # take out the error signal from vision
        steer_error = np.array(local_err) * np.pi  # take out the error signal from vision
        q = steer_error[0] * 0.7  # scale normalized input back to original range

        # arctan2 input set this way to account for different alignment
        # of (x, y) axes of rover and the environment
        # turn_des = np.arctan2(-error[0], abs(error[1]))
        turn_des = np.arctan2(-steer_error[0], steer_error[1])
        u1 = steer_scale * (turn_des - q) / 2  # divide by 2 to get in -1 to 1 ish range

        accel_err = [local_err[1], feedback['rgb'].flatten()]
        u2 = accel_scale * min(np.linalg.norm(-local_err), 1)
        u = [u1, u2]
        interface.send_forces(u)
        total_time += dt
        step += 1

        if step % camera_vis_fps == 0:
            plt.figure()
            plt.imshow(feedback['rgb']/255)
            plt.show()

except:
    print(traceback.format_exc())
    interface.disconnect()

finally:
    fig0 = plt.figure()
    target = np.array(target_track)
    rover = np.array(rover_track)
    plt.plot(target[:, 0], target[:, 1], "x", mew=3)
    plt.plot(rover[:, 0], rover[:, 1], lw=2)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.gca().set_aspect("equal")
    plt.title("Rover trajectory in environment")
    fig0.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig("figures/rover_trajectory.pdf")
    plt.show()
