import numpy as np
import time

import abr_control

# initialize our robot config for neural controllers
robot_config = abr_control.arms.jaco2.config.robot_config()
# instantiate the REACH controller for the ur5 robot
#ctrlr = abr_control.controllers.osc_robust.controller(robot_config)
# create our VREP interface for the ur5
interface = abr_control.interfaces.jaco2.interface(robot_config)
interface.connect()

try:
    while 1:
        interface.apply_u(np.zeros(6, dtype='float32'))

finally:
    interface.disconnect()
