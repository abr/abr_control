"""
Move the jao2 Mujoco arm to a target position.
The simulation ends after 1500 time steps, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np
import traceback
import glfw

from abr_control.controllers import OSC, Damping
from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.interfaces.mujoco import Mujoco
from abr_control.utils import transformations


# initialize our robot config
robot_config = arm('jaco2', use_sim_state=False)

# create our Mujoco interface
interface = Mujoco(robot_config, dt=.001)
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
    ctrlr_dof = [True, True, True, False, False, False])

interface.send_target_angles(robot_config.START_ANGLES)

# set up lists for tracking data
ee_track = []
target_track = []

target_geom_id = interface.sim.model.geom_name2id("target")
green = [0, 0.9, 0, 0.5]
red = [0.9, 0, 0, 0.5]

def gen_target(interface):
    target_xyz = (np.random.rand(3) + np.array([-0.5, -0.5, 0.75])) * np.array([1, 1, .5])
    interface.set_mocap_xyz(name='target', xyz=target_xyz)

try:
    # get the end-effector's initial position
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', feedback['q'])

    # make the target offset from that start position
    gen_target(interface)

    count = 0.0
    print('\nSimulation starting...\n')
    while 1:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()

        target = np.hstack([
            interface.get_xyz('target'),
            transformations.euler_from_quaternion(
                interface.get_orientation('target'), 'rxyz')])

        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            )

        # add gripper forces
        u = np.hstack((u, np.ones(3)*0.05))

        # send forces into Mujoco, step the sim forward
        interface.send_forces(u)

        # calculate end-effector position
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])
        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target[:3]))

        error = np.linalg.norm(ee_xyz-target[:3])
        if error < 0.02:
            interface.sim.model.geom_rgba[target_geom_id] = green
            count += 1
        else:
            count = 0
            interface.sim.model.geom_rgba[target_geom_id] = red

        if count >= 50:
            gen_target(interface)
            count = 0

except:
    print(traceback.format_exc())

finally:
    # stop and reset the Mujoco simulation
    interface.disconnect()

    print('Simulation terminated...')

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        fig = plt.figure(figsize=(8,12))
        ax1 = fig.add_subplot(211)
        ax1.set_ylabel('Distance (m)')
        ax1.set_xlabel('Time (ms)')
        ax1.set_title('Distance to target')
        ax1.plot(np.sqrt(np.sum((np.array(target_track) -
                                 np.array(ee_track))**2, axis=1)))

        ax2 = fig.add_subplot(212, projection='3d')
        ax2.set_title('End-Effector Trajectory')
        ax2.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label='ee_xyz')
        ax2.scatter(target_track[0, 0], target_track[0, 1], target_track[0, 2],
                 label='target', c='r')
        ax2.legend()
        plt.show()
