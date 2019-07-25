"""
Running operational space control using Mujoco. The controller will
move the end-effector to the target object's orientation.
"""
import numpy as np
import glfw

from abr_control.controllers import OSC, Damping
from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.interfaces.mujoco import Mujoco
from abr_control.utils import transformations


# initialize our robot config
robot_config = arm('jaco2')

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create opreational space controller
ctrlr = OSC(
    robot_config,
    kp=200,  # position gain
    ko=200,  # orientation gain
    null_controllers=[damping],
    # control (alpha, beta, gamma) out of [x, y, z, alpha, beta, gamma]
    ctrlr_dof = [False, False, False, True, True, True])

# create our interface
interface = Mujoco(robot_config, dt=.005)
interface.connect()

# set up lists for tracking data
ee_angles_track = []
target_angles_track = []


try:
    print('\nSimulation starting...\n')
    def get_random_orientation():
        rand_orient = np.random.rand(4)
        rand_orient /= np.linalg.norm(rand_orient)
        return rand_orient
    cnt = 0
    while 1:
        if cnt % 500 == 0:
            # generate a random target orientation
            rand_orient = get_random_orientation()
            print('New target orientation: ', rand_orient)

        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break

        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        # set our target to our ee_xyz since we are only focussing on orinetation
        interface.set_mocap_xyz('target_orientation', hand_xyz)
        interface.set_mocap_orientation('target_orientation', rand_orient)

        target = np.hstack([
            interface.get_mocap_xyz('target_orientation'),
            transformations.euler_from_quaternion(
                interface.get_mocap_orientation('target_orientation'), 'rxyz')])

        rc_matrix = robot_config.R('EE', feedback['q'])
        rc_angles = transformations.euler_from_matrix(rc_matrix, axes='rxyz')

        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            )

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # track data
        ee_angles_track.append(transformations.euler_from_matrix(
            robot_config.R('EE', feedback['q']), axes='rxyz'))
        target_angles_track.append(transformations.euler_from_quaternion(
                interface.get_mocap_orientation('target_orientation'), 'rxyz'))
        cnt += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')

    ee_angles_track = np.array(ee_angles_track)
    target_angles_track = np.array(target_angles_track)

    if ee_angles_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt

        plt.figure()
        plt.plot(ee_angles_track)
        plt.gca().set_prop_cycle(None)
        plt.plot(target_angles_track, '--')
        plt.ylabel('3D orientation (rad)')
        plt.xlabel('Time (s)')
        plt.tight_layout()
        plt.show()
