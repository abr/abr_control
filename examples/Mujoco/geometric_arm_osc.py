"""
Minimal example using the mujoco interface with a three link arm
reaching to a target with three degrees of freedom

To plot the joint angles call the script with True
passed through sys.argv
    python threelink.py True
"""

import glfw
import sys
import numpy as np

import mujoco_py as mjp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from abr_control.controllers import OSC
from abr_control.interfaces.mujoco import Mujoco
from abr_control.arms.mujoco_config import MujocoConfig


print('****************************************************************'
      + '***************************************')
print('\n***Append 1, 2, or 3 to your function call to change between a'
      + ' onejoint, twojoint, or threejoint arm***\n')
print('****************************************************************'
      + '***************************************')
show_plot = False
if len(sys.argv) > 1:
    N_JOINTS = int(sys.argv[1])
else:
    N_JOINTS = 3

if N_JOINTS == 1:
    model_filename = 'onejoint'
    ctrlr_dof=[True, False, False, False, False, False]
elif N_JOINTS == 2:
    model_filename = 'twojoint'
    ctrlr_dof=[True, True, False, False, False, False]
elif N_JOINTS == 3:
    model_filename = 'threejoint'
    ctrlr_dof=[True, True, True, False, False, False]
else:
    raise Exception ("Only 1-3 joint arms are available in this example")


robot_config = MujocoConfig(model_filename)

dt = 0.001
interface = Mujoco(robot_config, dt=dt)
interface.connect()

ctrlr = OSC(robot_config, kp=10, kv=5,
            ctrlr_dof=ctrlr_dof)

interface.send_target_angles(np.ones(N_JOINTS))

target = np.array([0.1, 0.1, 0.3, 0, 0, 0])
interface.set_mocap_xyz('target', target[:3])

interface.set_mocap_xyz('hand', np.array([.2, .4, 1]))

q_track = []
count = 0

link_name = 'EE'
try:
    while True:

        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break

        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx(link_name)
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            ref_frame=link_name,
            )
        interface.send_forces(u)

        if np.linalg.norm(hand_xyz[:N_JOINTS] - target[:N_JOINTS]) < 0.01:
            target[0] = (np.random.uniform(0.2, 0.25)
                         * np.sign(np.random.uniform(-1, 1)))
            target[1] = (np.random.uniform(0.2, 0.25)
                         * np.sign(np.random.uniform(-1, 1)))
            target[2] = np.random.uniform(0.4, 0.5)
            interface.set_mocap_xyz('target', target[:3])

        q_track.append(feedback['q'])
        count += 1

finally:
    interface.disconnect()

    q_track = np.asarray(q_track)

    if show_plot:
        plt.figure(figsize=(30, 30))

        plt.plot(q_track)
        plt.ylabel('Joint Angles [rad]')
        plt.legend(['0', '1'])

        plt.show()
