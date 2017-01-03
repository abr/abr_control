import numpy as np
import time

import abr_control

# initialize our robot config for neural controllers
robot_config = abr_control.arms.jaco2.config.robot_config()
# create our VREP interface for the ur5
interface = abr_control.interfaces.jaco2.interface(robot_config)

try:
    interface.connect()

except Exception as e:
    print(e)

finally:
    interface.disconnect()
