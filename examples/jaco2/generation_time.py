"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1.5 simulated seconds, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np

import abr_control

# initialize our robot config for the ur5
robot_config = abr_control.arms.jaco2.config(
    regenerate_functions=True)

# instantiate controller
ctrlr = abr_control.controllers.osc(
    robot_config, kp=200, vmax=0.5)

# create a target based on initial arm position
q = np.zeros(6)
start = robot_config.Tx('EE', q=q)
target_xyz = start + np.array([-.25, .25, 0])

import time
start_time = time.time()
# generate control signal
u = ctrlr.control(
    q=q,
    dq=np.zeros(6),
    target_x=target_xyz,
    target_dx=np.zeros(3))
print('time: %.6f' % (time.time() - start_time))
