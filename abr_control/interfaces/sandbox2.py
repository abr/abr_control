import numpy as np
import time

import abr_control

# initialize our robot config for neural controllers
robot_config = abr_control.arms.jaco2.config.robot_config()
# instantiate the REACH controller for the ur5 robot
ctrlr = abr_control.controllers.osc_robust.controller(
  robot_config, kp=600, vmax=0.35)
# run controller once to generate functions / take care of overhead
# outside of the main loop (so the torque mode isn't exited)
ctrlr.control(np.zeros(6), np.zeros(6), target_state=np.zeros(6))
# create our VREP interface for the ur5
interface = abr_control.interfaces.jaco2.interface(robot_config)
interface.connect()
interface.apply_q()
interface.init_force_mode()

start_comm = 0
end_comm =  0
comm_speed = []
start_loop = 0
end_loop = 0
loop_speed = []

q_track = []
dq_track = []
ee_track = []
targets = []
targets_track = []
error_track = []
count = 0
target_index = 0
at_target_count = 0

target_xyz = [-.467, .22, .78]
print('Moving to first target: ', target_xyz)

try:
    while 1:
        start_loop = time.time()
        feedback = interface.get_feedback()
        q = (np.array(feedback['q']) % 360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0
        u = ctrlr.control(q=q, dq=dq,
                          target_state=np.hstack([target_xyz,
                                                 np.zeros(3)]))
        end_loop = time.time()
        loop_speed.append(end_loop - start_loop)
        

        #print('u: ', u)
        #u = np.around(u, 1)
        #u = np.zeros(6)
        start_comm = time.time()
        interface.apply_u(np.array(u, dtype='float32'))
        end_comm = time.time()
        comm_speed.append(end_comm - start_comm)

        hand_xyz = robot_config.T('EE', q=q)
        error = np.sqrt(np.sum((hand_xyz - target_xyz)**2))
        # print('error: ', error)
        if error < .01:
            at_target_count += 1
            if at_target_count >= 200:
                break

        ee_track.append(hand_xyz)
        targets_track.append(target_xyz)
        error_track.append(error)
        count += 1
        if count == 1000:
            break
        
        
except Exception as e:
    print(e)

finally:
    interface.init_position_mode()
    interface.apply_q()
    interface.disconnect()
    print(u)

    np.savez_compressed('data/4.2/ee', ee=ee_track)
    np.savez_compressed('data/4.2/targets', targets=targets_track)

    if count > 0:
        import matplotlib.pyplot as plt
        import seaborn

        """plt.subplot(2, 1, 1)
        plt.plot(q_track)
        plt.legend(range(6))
        plt.title('Joint angles')

        plt.subplot(2, 1, 2)
        plt.plot(dq_track[10:])
        plt.legend(range(6))
        plt.title('Joint velocities')

        plt.figure()
        plt.plot(error_track)
        plt.ylabel('Euclidean distance to target (m)')
        plt.xlabel('Time steps')
        plt.title('Error over time')"""
        
        plt.figure()
        plt.plot(list(range(len(loop_speed))), loop_speed, lw=2, label = 'Controller Speed', color = 'green') # lw sets line width
        plt.plot(list(range(len(comm_speed))), comm_speed, lw=2, label = 'Comm Speed', color = 'red')
        plt.legend(bbox_to_anchor=(1, 1), loc=1)
        plt.xlabel('Loop Counter')
        plt.ylabel('Time')
        plt.title('Loop and Comm Speeds')

        """ee_track = np.array(ee_track)
        targets_plot = np.ones(ee_track.shape) * target_xyz
        for ii in range(len(targets)):
            targets_plot[ii] = targets[ii]
        abr_control.utils.plotting.plot_trajectory(ee_track, targets_plot)

        plt.tight_layout()"""
        print("Average Comm Speed: ")
        print(sum(comm_speed)/len(comm_speed))
        plt.show()
