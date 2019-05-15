"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1.5 simulated seconds, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np
import traceback

from abr_control.arms import jaco2 as arm
from abr_control.controllers import OSC, Damping, signals
from abr_control.interfaces import VREP

# initialize our robot config for the jaco2
robot_config = arm.Config(use_cython=True, hand_attached=False)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# instantiate controller
ctrlr = OSC(robot_config, kp=200, null_controllers=[damping],
            vmax=[0.5, 0],  # [m/s, rad/s]
            # control (x, y, z) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof = [True, True, True, False, False, False])

# create our adaptive controller
adapt = signals.DynamicsAdaptation(
    n_neurons=1000,
    n_ensembles=5,
    n_input=2,  # we apply adaptation on the most heavily stressed joints
    n_output=2,
    pes_learning_rate=5e-5,
    intercepts_bounds=[-0.6, -0.2],
    intercepts_mode=-0.2,
    means=[3.14, 3.14],
    variances=[1.57, 1.57])

# create our VREP interface
interface = VREP(robot_config, dt=.005)
interface.connect()

# set up lists for tracking data
ee_track = []
target_track = []

# get Jacobians to each link for calculating perturbation
J_links = [robot_config._calc_J('link%s' % ii, x=[0, 0, 0])
           for ii in range(robot_config.N_LINKS)]


try:
    # get the end-effector's initial position
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', feedback['q'])

    # make the target offset from that start position
    target_xyz = start + np.array([0.2, -0.2, 0.0])
    interface.set_xyz(name='target', xyz=target_xyz)

    count = 0.0
    while 1:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()

        target = np.hstack([
            interface.get_xyz('target'),
            interface.get_orientation('target')])

        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            )

        u_adapt = np.zeros(robot_config.N_JOINTS)
        u_adapt[1:3] = adapt.generate(
            input_signal=np.array(
                [feedback['q'][1], feedback['q'][2]]),
            training_signal=np.array(
                [ctrlr.training_signal[1], ctrlr.training_signal[2]]))
        u += u_adapt

        # add an additional force for the controller to adapt to
        extra_gravity = robot_config.g(feedback['q']) * 2
        u += extra_gravity

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
