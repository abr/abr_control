"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1.5 simulated seconds, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np

import abr_control

import os;
os.environ["SYMPY_USE_CACHE"] = "yes"
os.environ["SYMPY_CACHE_SIZE"] = "None"

# initialize our robot config for the ur5
robot_config = abr_control.arms.jaco2.config(
    regenerate_functions=True, use_simplify=True)

# instantiate controller
ctrlr = abr_control.controllers.osc(
    robot_config, kp=200, vmax=0.5)

# create a target based on initial arm position
q = np.zeros(6)

import time
start_time = time.time()
# generate control signal
# u = ctrlr.control(
#     q=q,
#     dq=np.zeros(6),
#     target_x=np.zeros(3),
#     target_dx=np.zeros(3))
# print('control time: %.6f' % (time.time() - start_time))
name = 'EE'
# x = robot_config.Tx(name, q=np.random.random(6),
#                     x=[.1, .1, .1])
# print('Tx time: %.6f' % (time.time() - start_time))

num = 2
times = np.empty(num)
for ii in range(num):
    print('trial ', ii)
    start_time = time.time()
    J = robot_config._calc_J(name, x=[0, 0, 0], regenerate=True)
    times[ii] = (time.time() - start_time)
print('Average J time: %.5f' % (np.sum(times) / num))
