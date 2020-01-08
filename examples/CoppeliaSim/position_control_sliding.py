"""
Move the UR5 CoppeliaSim arm to a target position.
The simulation ends after 1500 time steps, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np
import traceback

# from abr_control.arms import ur5 as arm
from abr_control.arms import jaco2 as arm
# from abr_control.arms import onejoint as arm
from abr_control.controllers import Sliding
from abr_control.interfaces import CoppeliaSim

# initialize our robot config
robot_config = arm.Config()

# instantiate controller
# NOTE: These values are non-optimal
ctrlr = Sliding(robot_config, kd=10.0, lamb=30.00)

# create our CoppeliaSim interface
interface = CoppeliaSim(robot_config, dt=.001)
interface.connect()

# set up lists for tracking data
ee_track = []
target_track = []


try:
    # get the end-effector's initial position
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', feedback['q'])

    # make the target offset from that start position
    target_xyz = start + np.array([0.2, -0.2, 0.0])
    interface.set_xyz(name='target', xyz=target_xyz)

    count = 0.0
    print('\nSimulation starting...\n')
    while count < 1500:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()

        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target_xyz
            )

        # send forces into CoppeliaSim, step the sim forward
        interface.send_forces(u)

        # calculate end-effector position
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])
        # track data
        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target_xyz))

        count += 1

except:
    print(traceback.format_exc())

finally:
    # stop and reset the CoppeliaSim simulation
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
