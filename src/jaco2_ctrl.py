import numpy as np
from abr_control.arms import jaco2 as arm
from abr_control.controllers import OSC
from abr_control.interfaces import VREP

# initialize our robot config for the ur5
robot_config = arm.Config(use_cython=True, hand_attached=True)

# instantiate controller
ctrlr = OSC(robot_config, kp=200, vmax=0.5)

# create our VREP interface
interface = VREP(robot_config, dt=.0005)
interface.connect()

target_xyz = np.array([0.5, 0.2, 0.5])
# set the target object's position in VREP
interface.set_xyz(name='target', xyz=target_xyz)

count = 0.0
while count < 1500:  # run for 1.5 simulated seconds
    # get joint angle and velocity feedback
    feedback = interface.get_feedback()
    # calculate the control signal
    u = ctrlr.generate(
        q=feedback['q'],
        dq=feedback['dq'],
        target_pos=target_xyz)
    # send forces into VREP, step the sim forward
    interface.send_forces(u)
    count += 1
interface.disconnect()