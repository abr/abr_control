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
robot_config = arm.Config(use_cython=True, hand_attached=True)

# get Jacobians to each link for calculating perturbation
J_links = [robot_config._calc_J('link%s' % ii, x=[0, 0, 0])
           for ii in range(robot_config.N_LINKS)]

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# instantiate controller
ctrlr = OSC(robot_config, kp=200, vmax=0.5, null_controllers=[damping])

# create our adaptive controller
adapt = signals.DynamicsAdaptation(
    backend='nengo',
    n_neurons=5000,
    n_input=2,  # we apply adaptation on the most heavily stressed joints
    n_output=2,
    weights_file=None,
    pes_learning_rate=1e-2,
    intercepts=(-.9, -.2))

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
    target_xyz = start + np.array([0.2, -0.2, 0.0])
    interface.set_xyz(name='target', xyz=target_xyz)

    count = 0.0
    while 1:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()
        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_xyz)

        scaled = robot_config.scaledown('q', feedback['q'])
        u_adapt = adapt.generate(
            input_signal=np.array([scaled[1], scaled[2]]),
            training_signal=np.array(
                [ctrlr.training_signal[1], ctrlr.training_signal[2]]))
        u_adapt = np.array([0, u_adapt[0], u_adapt[1], 0, 0, 0])
        u += u_adapt
        print('u_adapt: ', [float('%.3f' % val) for val in u_adapt])

        # add an additional force for the controller to adapt to
        extra_gravity = robot_config.g(feedback['q']) * 2
        # g = np.zeros((robot_config.N_JOINTS, 1))
        # for ii in range(robot_config.N_LINKS):
        #     pars = tuple(feedback['q']) + tuple([0, 0, 0])
        #     g += np.dot(J_links[ii](*pars).T, fake_gravity)
        u += extra_gravity

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
