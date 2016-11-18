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

q_track = []
dq_track = []

try:
    count = 0
    u = np.zeros(6)
    for ii in range(2000):
        u[0] = 4.0 * np.sin(2.0*np.pi * ii/2000);

        interface.apply_u(np.array(u, dtype='float32'))
        feedback = interface.get_feedback()
        q_track.append(feedback['q'])
        dq_track.append(feedback['dq'])

finally:
    interface.disconnect()

    import matplotlib.pyplot as plt
    import seaborn

    plt.subplot(2, 1, 1)
    plt.plot(q_track)

    plt.subplot(2, 1, 2)
    plt.plot(dq_track)

    plt.tight_layout()
    plt.show()
