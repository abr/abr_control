"""
Running the threelink arm with a pygame display, and using the pydmps
library to specify a trajectory for the end-effector to follow, in 
this case, a circle. The program will run until ctrl-C is called.
"""
import numpy as np

import abr_control
import pydmps

# create a dmp that traces a circle
x = np.linspace(0, np.pi*2, 100)
dmps_traj = np.array([np.cos(x)*.5, np.sin(x)*.5 + 2])
dmps = pydmps.DMPs_rhythmic(n_dmps=2, n_bfs=50, dt=.01)
dmps.imitate_path(dmps_traj)

# initialize our robot config for the ur5
robot_config = abr_control.arms.threelink.config.robot_config(
    regenerate_functions=False)

# create an operational space controller
ctrlr = abr_control.controllers.osc.controller(
    robot_config, kp=500, vmax=None)

# create our interface
interface = abr_control.interfaces.maplesim.interface(
    robot_config, dt=.001)
interface.connect()

# create a target
feedback = interface.get_feedback()
target_xyz = robot_config.Tx('EE', feedback['q'])
interface.set_target(target_xyz)

# set up lists for tracking data
ee_path = []
target_path = []

try:
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', feedback['q'])
        error = np.sqrt(np.sum((target_xyz - hand_xyz)**2))

        # generate an operational space control signal
        u = ctrlr.control(
            q=feedback['q'],
            dq=feedback['dq'],
            target_state=np.hstack(
                [target_xyz, np.zeros(3)]))

        # get the next point in the target trajectory from the dmp
        target_xyz[0], target_xyz[1] = dmps.step(
            error=error*1e2)[0]
        interface.set_target(target_xyz)

        # apply the control signal, step the sim forward
        interface.apply_u(u)

        # track data
        ee_path.append(np.copy(hand_xyz))
        target_path.append(np.copy(target_xyz))

finally:
    # stop and reset the simulation
    interface.disconnect()
