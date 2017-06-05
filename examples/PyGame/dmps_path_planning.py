"""
Running the threelink arm with a PyGame display, and using the pydmps
library to specify a trajectory for the end-effector to follow, in
this case, a circle.

To install the pydmps library, clone https://github.com/studywolf/pydmps
and run 'python setup.py develop'
"""
import numpy as np

try:
    import pydmps
except ImportError:
    print('\npydmps library required, see ' +
          'https://github.com/studywolf/pydmps\n')

from abr_control.arms import threelink as arm
# from abr_control.arms import twolink as arm
from abr_control.interfaces import PyGame
from abr_control.controllers import OSC


# create a dmp that traces a circle
x = np.linspace(0, np.pi*2, 100)
dmps_traj = np.array([np.cos(x)*.5, np.sin(x)*.5 + 2])
dmps = pydmps.DMPs_rhythmic(n_dmps=2, n_bfs=50, dt=.01)
dmps.imitate_path(dmps_traj)

# initialize our robot config
robot_config = arm.Config(use_cython=True)

# create our arm simulation
arm_sim = arm.ArmSim(robot_config)

# create an operational space controller
ctrlr = OSC(robot_config, kp=500, vmax=20)

# create our interface
interface = PyGame(robot_config, arm_sim, dt=.001)
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
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target_pos=target_xyz)

        # get the next point in the target trajectory from the dmp
        target_xyz[0], target_xyz[1] = dmps.step(
            error=error*1e2)[0]
        interface.set_target(target_xyz)

        # apply the control signal, step the sim forward
        interface.send_forces(u)

        # track data
        ee_path.append(np.copy(hand_xyz))
        target_path.append(np.copy(target_xyz))

finally:
    # stop and reset the simulation
    interface.disconnect()
