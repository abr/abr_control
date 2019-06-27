import glfw
import sys
import numpy as np

import mujoco_py as mjp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from abr_control.controllers import OSC, Damping, RestingConfig
from abr_control.interfaces import Mujoco
from abr_control.arms import MujocoConfig


if len(sys.argv) > 1:
    show_plot = sys.argv[1]
else:
    show_plot = False
# NOTE: the meshes file from MuJoCo_Unity_UR5/xmls needs to be in the
# Mujoco examples folder for this to load properly
model_filename = 'jaco2_threelink.xml'
q_init = np.array([0, 0, 0])#, -np.pi/2, np.pi/2])

robot_config = MujocoConfig(model_filename)

dt = 0.005
interface = Mujoco(robot_config, dt=dt)
interface.connect()

ctrlr = OSC(robot_config, kp=30, kv=20,
            ctrlr_dof=[True, True, True, False, False, False])

interface.send_target_angles(q_init)

target = np.array([-0.2, 0.2, 0.3, 0, 0, 0])
interface.set_mocap_xyz('target', target[:3])

interface.set_mocap_xyz('hand', np.array([.2, .4, 1]))

q_track = []
u_track = []
# ee_track = []
# target_track = []

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

        if count %500 == 0:
            print('ee_xyz: ', hand_xyz)
            print('target', target)
            print('u: ', u)
            print('q: ', feedback['q'])

        if np.linalg.norm(hand_xyz[:3] - target[:3]) < 0.01:
            target[0] = np.random.uniform(0.2, 0.3) * np.sign(np.random.uniform(-1, 1))
            target[1] = np.random.uniform(0.2, 0.25) * np.sign(np.random.uniform(-1, 1))
            target[2] = np.random.uniform(0.3, 0.4)
            interface.set_mocap_xyz('target', target[:3])

        q_track.append(feedback['q'])
        u_track.append(u)
        # ee_track.append(np.copy(hand_xyz))
        # target_track.append(interface.get_mocap_xyz(link_name))
        count += 1

finally:
    interface.disconnect()

    q_track = np.asarray(q_track)
    # ee_track = np.array(ee_track)
    # target_track = np.array(target_track)

    if show_plot:
        plt.figure(figsize=(30, 30))

        plt.subplot(211)
        plt.plot(q_track)
        plt.ylabel('Joint Angles [rad]')
        plt.legend(['0', '1'])

        plt.subplot(212)
        plt.plot(u_track)
        plt.ylabel('Joint Force [Nm]')
        plt.legend(['0', '1'])
        # plt.subplot(2, 1, 1, projection='3d')
        # plt.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2])
        # plt.plot(target_track[:, 0], target_track[:, 1], target_track[:, 2], 'rx', mew=3, lw=2)
        # plt.gca().set_xlim([-1, 1])
        # plt.gca().set_ylim([-1, 1])
        # plt.gca().set_zlim([0, 1])
        # plt.gca().set_aspect('equal')
        #
        # plt.subplot(2, 1, 2)
        # plt.plot(ee_track, lw=2)
        # plt.gca().set_prop_cycle(None)
        # plt.plot(target_track, '--', lw=2)

        plt.show()
