"""
Running operational space control using CoppeliaSim. The controller will
move the end-effector to the target object's position and orientation.
"""
import numpy as np

from abr_control.arms import ur5 as arm
from abr_control.controllers import OSC, Damping
from abr_control.interfaces import CoppeliaSim
from abr_control.utils import transformations


# initialize our robot config
robot_config = arm.Config()

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create opreational space controller
ctrlr = OSC(
    robot_config,
    kp=200,
    null_controllers=[damping],
    vmax=[10, 10],  # [m/s, rad/s]
    # control (x, alpha, beta, gamma) out of [x, y, z, alpha, beta, gamma]
    ctrlr_dof=[True, False, False, True, True, True])

# create our interface
interface = CoppeliaSim(robot_config, dt=.005)
interface.connect()

# set up lists for tracking data
ee_track = []
ee_angles_track = []
target_track = []
target_angles_track = []


try:
    count = 0
    print('\nSimulation starting...\n')
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        target = np.hstack([
            interface.get_xyz('target'),
            interface.get_orientation('target')])

        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            )

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # track data
        ee_track.append(np.copy(hand_xyz))
        ee_angles_track.append(transformations.euler_from_matrix(
            robot_config.R('EE', feedback['q']), axes='rxyz'))
        target_track.append(np.copy(target[:3]))
        target_angles_track.append(interface.get_orientation('target'))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')

    ee_track = np.array(ee_track)
    ee_angles_track = np.array(ee_angles_track)
    target_track = np.array(target_track)
    target_angles_track = np.array(target_angles_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

        fig = plt.figure(figsize=(8,12))
        ax1 = fig.add_subplot(311)
        ax1.set_ylabel('3D position (m)')
        ax1.plot(ee_track)
        ax1.plot(target_track, '--')

        ax2 = fig.add_subplot(312)
        ax2.plot(ee_angles_track)
        ax2.plot(target_angles_track, '--')
        ax2.set_ylabel('3D orientation (rad)')
        ax2.set_xlabel('Time (s)')

        ax3 = fig.add_subplot(313, projection='3d')
        ax3.set_title('End-Effector Trajectory')
        ax3.plot(ee_track[:, 0], ee_track[:, 1], ee_track[:, 2], label='ee_xyz')
        ax3.scatter(target_track[0, 0], target_track[0, 1], target_track[0, 2],
                    label='target', c='g')
        ax3.legend()
        plt.show()
