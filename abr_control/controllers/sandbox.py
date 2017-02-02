import numpy as np
import time

import abr_control

robot_config = abr_control.arms.jaco2.config(
    regenerate_functions=True)

print(np.array(robot_config._calc_T('joint0')))
print(np.array(robot_config._calc_T('joint1')))
print(robot_config.J_orientation[1])
