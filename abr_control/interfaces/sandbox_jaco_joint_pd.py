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
count = 0

try:
    target_q = 100.0
    kd = 1.0
    kv = np.sqrt(kd)
    u = np.zeros(6)
    while 1:
        feedback = interface.get_feedback()
        u[5] = kd * (target_q - feedback['q'][5]) - kv * feedback['dq'][5] 
        #print('u: ', u)
        interface.apply_u(np.array(u, dtype='float32'))
        q_track.append(feedback['q'])
        dq_track.append(feedback['dq'])
        count += 1
        
finally:
    interface.disconnect()

    if count > 0: 
        import matplotlib.pyplot as plt
        import seaborn

        plt.subplot(2, 1, 1)
        plt.plot(q_track)
        plt.legend(range(6))
        plt.title('Joint angles')
        
        plt.subplot(2, 1, 2)
        plt.plot(dq_track[10:])
        plt.legend(range(6))
        plt.title('Joint velocities')

        plt.tight_layout()
        plt.show()
