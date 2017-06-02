"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1.5 simulated seconds, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np
import traceback

# from abr_control.arms import ur5 as arm
from abr_control.arms import jaco2 as arm
# from abr_control.arms import onelink as arm
from abr_control.controllers import OSC
from abr_control.interfaces import VREP

# initialize our robot config for the ur5
robot_config = arm.Config(use_cython=True)

# instantiate controller
ctrlr = OSC(robot_config, kp=200, vmax=0.5)

# create our VREP interface
interface = VREP(robot_config, dt=.001)
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
    while count < 1500:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()
        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_xyz)
        # send forces into VREP, step the sim forward
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
    # stop and reset the VREP simulation
    interface.disconnect()

    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    if ee_track.shape[0] > 0:
        # plot distance from target and 3D trajectory
        import matplotlib.pyplot as plt
        from abr_control.utils.plotting import plot_3D

        plt.figure()
        plt.plot(np.sqrt(np.sum((np.array(target_track) -
                                np.array(ee_track))**2, axis=1)))
        plt.ylabel('Distance (m)')
        plt.xlabel('Time (ms)')
        plt.title('Distance to target')

        plot_3D(ee_track, target_track)
        plt.show()
