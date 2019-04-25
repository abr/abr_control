"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1500 time steps, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np
import traceback

from abr_control.arms import ur5 as arm
# from abr_control.arms import jaco2 as arm
# from abr_control.arms import onelink as arm
from abr_control.controllers import OSC, Damping
from abr_control.interfaces import VREP
from abr_control.utils import transformations

# initialize our robot config
robot_config = arm.Config(use_cython=True)
# if using the Jaco 2 arm with the hand attached, use the following instead:
# robot_config = arm.Config(use_cython=True, hand_attached=True)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# instantiate controller
ctrlr = OSC(robot_config, kp=200, null_controllers=[damping],
            vmax=[0.5, 0],  # [m/s, rad/s]
            # control (x, y, z) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof = [True, True, True, False, False, False])


# create our VREP interface
interface = VREP(robot_config, dt=.005)
interface.connect()

# set up lists for tracking data
ee_track = []
target_track = []


try:
    # get the end-effector's initial position
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', feedback['q'])

    # make the target offset from that start position
    target_xyz = start + np.array([0.2, -0.2, -0.3])
    interface.set_xyz(name='target', xyz=target_xyz)

    count = 0.0
    print('\nSimulation starting...\n')
    while count < 1500:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()

        target = np.hstack([
            interface.get_xyz('target'),
            interface.get_orientation('target')])

        name = 'joint1'
        vrep_angles = interface.get_orientation('UR5_%s' % name)

        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            )

        # send forces into VREP, step the sim forward
        interface.send_forces(u)

        # calculate end-effector position
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])
        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target[:3]))

        count += 1

except:
    print(traceback.format_exc())

finally:
    # stop and reset the VREP simulation
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
