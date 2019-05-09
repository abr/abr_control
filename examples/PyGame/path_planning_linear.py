"""
Running the operational space control with a first order path planner
using the PyGame display. The path planner system will generate a
trajectory for the controller to follow, moving the end-effector in a
straight line to the target, which changes every n time steps.
"""
import numpy as np

from abr_control.arms import threejoint as arm
# from abr_control.arms import twojoint as arm
from abr_control.interfaces import PyGame
from abr_control.controllers import OSC, Damping, path_planners


# initialize our robot config
robot_config = arm.Config(use_cython=True)

# create our arm simulation
arm_sim = arm.ArmSim(robot_config)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create an operational space controller
ctrlr = OSC(robot_config, kp=100, null_controllers=[damping],
            # control (gamma) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof = [True, True, False, False, False, False])

# create our path planner
n_timesteps = 250  # give 250 time steps to reach target
path_planner = path_planners.Linear()

# create our interface
interface = PyGame(robot_config, arm_sim, dt=.001)
interface.connect()


try:
    print('\nSimulation starting...')
    print('Click to move the target.\n')

    count = 0
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        if count % n_timesteps == 0:
            target_xyz = np.array([
                np.random.random() * 2 - 1,
                np.random.random() * 2 + 1,
                0])
            # update the position of the target
            interface.set_target(target_xyz)
            path_planner.generate_path(
                position=hand_xyz,
                target_pos=target_xyz,
                # n_timesteps=n_timesteps,
                dx=0.01,
                plot=False)

        # returns desired [position, velocity]
        target = path_planner.next_target()

        # generate an operational space control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=np.hstack([target[:3], np.zeros(3)]),
            target_vel=np.hstack([target[3:], np.zeros(3)]),
            )

        # apply the control signal, step the sim forward
        interface.send_forces(
            u, update_display=True if count % 20 == 0 else False)

        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')
