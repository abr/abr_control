"""
Running the operational space control with a first order path planner
using the PyGame display. The path planner system will generate a
trajectory for the controller to follow, moving the end-effector in a
straight line to the target, which changes every n time steps.
"""
import numpy as np

import matplotlib.pyplot as plt

from abr_control.arms import threejoint as arm
# from abr_control.arms import twojoint as arm
from abr_control.interfaces.pygame import PyGame
from abr_control.controllers import OSC, Damping, path_planners


# initialize our robot config
robot_config = arm.Config()

# create our arm simulation
arm_sim = arm.ArmSim(robot_config)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create an operational space controller
ctrlr = OSC(robot_config, kp=200, null_controllers=[damping],
            # control (gamma) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof = [True, True, False, False, False, False])

# create our path planner
target_dx = 0.001
path_planner = path_planners.Linear(dx=target_dx)

# create our interface
interface = PyGame(robot_config, arm_sim, dt=0.001)
interface.connect()


try:
    print('\nSimulation starting...')
    print('Click to move the target.\n')

    count = 0
    dx_track = []
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])
        dx_track.append(np.linalg.norm(
            np.dot(robot_config.J('EE', feedback['q']), feedback['dq'])))

        if count % 1000 == 0:
            target_xyz = np.array([
                np.random.random() * 2 - 1,
                np.random.random() * 2 + 1,
                0])
            # update the position of the target
            interface.set_target(target_xyz)
            path_planner.generate_path(
                position=hand_xyz,
                target_position=target_xyz,
                plot=False)

        # returns desired [position, velocity]
        target, _ = path_planner.next()

        # generate an operational space control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=np.hstack([target, np.zeros(3)]),
            )

        # apply the control signal, step the sim forward
        interface.send_forces(
            u, update_display=True if count % 20 == 0 else False)

        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    dx_track = np.array(dx_track) * path_planner.dt
    plt.plot(dx_track, lw=2, label='End-effector velocity')
    plt.plot(np.ones(dx_track.shape) * target_dx, 'r--', lw=2,
             label='Target velocity')
    plt.ylabel('Velocity (m/s)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.tight_layout()
    plt.show()

    print('Simulation terminated...')
