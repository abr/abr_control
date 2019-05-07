"""
Running the operational space control with a second order path planner
using the PyGame display. The path planning system will generate
a trajectory for the controller to follow, moving the end-effector
smoothly to the target, which changes every n time steps.
"""
import numpy as np
import timeit

from abr_control.arms import threejoint as arm
# from abr_control.arms import twojoint as arm
from abr_control.interfaces import PyGame
from abr_control.controllers import OSC, Damping, path_planners


# initialize our robot config for the ur5
robot_config = arm.Config(use_cython=True)
# create our arm simulation
arm_sim = arm.ArmSim(robot_config)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create an operational space controller
ctrlr = OSC(robot_config, kp=200, null_controllers=[damping],
            # control (gamma) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof = [True, True, False, False, False, False])

# create our path planner
path_planner = path_planners.BellShaped(error_scale=50)

# create our interface
interface = PyGame(robot_config, arm_sim, dt=0.001)
interface.connect()

try:
    print('\nSimulation starting...')
    print('Click to move the target.\n')

    step_limit = 1000
    count = 1000
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        if count >= step_limit:
            count = 0
            target_xyz = (
                    np.array([
                        np.random.random() * 2 - 1,
                        np.random.random() * 2 + 1,
                        0]))
            # update the position of the target
            interface.set_target(target_xyz)

            path_planner.reset(target_pos=target_xyz, pos=hand_xyz)
            error = 0

        # returns desired [position, velocity]
        target, target_vel = path_planner.step(error=error)

        # generate an operational space control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=np.hstack((target, np.zeros(3))),
            target_vel=np.hstack((target_vel, np.zeros(3))),
            )

        # apply the control signal, step the sim forward
        interface.send_forces(
            u, update_display=True if count % 20 == 0 else False)

        error = np.sqrt(np.sum((hand_xyz - target)**2))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print('Simulation terminated...')
