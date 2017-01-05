import numpy as np
import time

import abr_control

# # initialize our robot config for the ur5
# robot_config = abr_control.arms.threelink.config.robot_config(
#     regenerate_functions=True)
#
# # test out our orientation calculations
# q = np.array([1, 1, 1])
# print(robot_config.orientation('EE', q=q))

# initialize our robot config for the ur5
# robot_config = abr_control.arms.ur5.config.robot_config(
robot_config = abr_control.arms.onelink.config.robot_config(
    regenerate_functions=True)

interface = abr_control.interfaces.vrep.interface(
    robot_config=robot_config, dt=.001)
interface.connect()

try:
    # test out our orientation calculations
    while 1:
        feedback = interface.get_feedback()
        print(robot_config.Tx('EE', q=feedback['q']))
        print(robot_config.orientation('EE', q=feedback['q']) * 180.0 / np.pi)
        time.sleep(1)

finally:
    # stop and reset the VREP simulation
    interface.disconnect()
