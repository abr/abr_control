import numpy as np
import abr_control

robot_config = abr_control.arms.threelink.config.robot_config()
interface = abr_control.interfaces.maplesim.interface(robot_config)

interface.connect()

u = np.ones(3) * 100

for ii in range(100):
    interface.apply_u(u)
