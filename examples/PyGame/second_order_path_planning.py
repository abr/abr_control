"""
Running the threelink arm with the PyGame display. The path planning will
system will generate a trajectory for the controller to follow, moving the
end-effector smoothly to the target, which changes every n time
steps.
"""
import numpy as np

import abr_control
from abr_control.arms.threelink.arm_sim import ArmSim
# from abr_control.arms.twolink.arm_sim import ArmSim
from abr_control.interfaces.pygame import PyGame

print('\nClick to move the target.\n')

# initialize our robot config for the ur5
robot_config = abr_control.arms.threelink.Config(use_cython=True)
# robot_config = abr_control.arms.twolink.Config(use_cython=True)

# create our arm simulation
arm_sim = ArmSim(robot_config)

# create an operational space controller
ctrlr = abr_control.controllers.OSC(
    robot_config, kp=100, vmax=10)

# create our path planner
n_timesteps = 100
path_planner = abr_control.controllers.path_planners.SecondOrder(
    robot_config)

# create our interface
interface = PyGame(robot_config, arm_sim, dt=.001)
interface.connect()

# set up lists for tracking data
ee_path = []
target_path = []

try:
    count = 0
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])

        #if count % n_timesteps == 0:
        if count == 0:
            target_xyz = np.array([
                np.random.random() * 2 - 1,
                np.random.random() * 2 + 1,
                0])
            # update the position of the target
            interface.set_target(target_xyz)
            path_planner.generate(state=hand_xyz, target=target_xyz,
                                  n_timesteps=n_timesteps)

        # returns desired [position, velocity]
        target = path_planner.next()

        # generate an operational space control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target[:3],
            target_vel=target[3:])

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # track data
        ee_path.append(np.copy(hand_xyz))
        target_path.append(np.copy(target_xyz))
        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()
