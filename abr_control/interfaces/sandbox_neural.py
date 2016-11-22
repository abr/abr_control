import matplotlib.pyplot as plt
import numpy as np
import seaborn
import subprocess
import time

import abr_control

# initialize our robot config for neural controllers
robot_config = abr_control.arms.jaco2.config_neural.robot_config()
# create our VREP interface for the ur5
interface = abr_control.interfaces.jaco2.interface(robot_config)

plotting = False

total_error_track = []

#start speech dispatcher
subprocess.call(['speech-dispatcher'])

for ii in range(0, 10):

    print('Iteration %i' % ii)
    interface.connect()

    q_track = []
    dq_track = []
    ee_track = []
    targets_track = []
    error_track = []
    count = 0
    target_index = 0
    at_target_count = 0

    targets = [[-.467, .22, .78],
               [-.467, -.22, .78],
               [.467, -.22, .78],
               [.467, .22, .78],
               [-.467, .22, .78]]
    target_xyz = targets[0]
    print('Moving to first target: ', target_xyz)
    save_history = 3

    try:

        print('Creating controller...')
        # instantiate the REACH controller for the ur5 robot
        ctrlr = abr_control.controllers.osc_adaptive.controller(
            robot_config, kp=600, vmax=0.35,
            pes_learning_rate=5e-5,
            voja_learning_rate=1e-5,
            weights_file='data/weights%i.npz' % ((ii - 1) % save_history),
            encoders_file='data/encoders%i.npz' % ((ii - 1) % save_history))
        print('Loading functions...')
        # run controller once to generate functions / take care of overhead
        # outside of the main loop (so the torque mode isn't exited)
        ctrlr.control(
            np.zeros(6),
            np.zeros(6),
            target_state=np.zeros(6),
            sim_neurons=False)

        print('Connecting interface...')

        subprocess.call(['spd-say', '"movement beginning in"'])
        time.sleep(1.5)
        subprocess.call(['spd-say', '"three"'])
        print('THREE')
        time.sleep(.5)
        subprocess.call(['spd-say', '"two"'])
        print('TWO')
        time.sleep(.5)
        subprocess.call(['spd-say', '"one"'])
        print('ONE')
        time.sleep(.5)

        interface.apply_q()
        interface.init_force_mode()
        while 1:
            feedback = interface.get_feedback()
            q = (np.array(feedback['q']) % 360) * np.pi / 180.0
            dq = np.array(feedback['dq']) * np.pi / 180.0
            u = ctrlr.control(
                q=q, dq=dq,
                target_state=np.hstack([target_xyz,
                                        np.zeros(3)]),
                sim_neurons=True)
            #print('u: ', u)
            interface.apply_u(np.array(u, dtype='float32'))

            hand_xyz = robot_config.T('EE', q=q)
            error = np.sqrt(np.sum((hand_xyz - target_xyz)**2))
            # print('error: ', error)
            if error < .01:
                at_target_count += 1
                if at_target_count >= 200:
                    target_index += 1
                    if target_index > len(targets):
                        break
                    else:
                        target_xyz = targets[target_index]
                        print('Moving to next target: ', target_xyz)
                    at_target_count = 0

            ee_track.append(hand_xyz)
            q_track.append(q)
            dq_track.append(dq)
            targets_track.append(target_xyz)
            error_track.append(error)
            count += 1

    except Exception as e:
        print(e)

    finally:
        interface.init_position_mode()
        interface.apply_q()
        interface.disconnect()

        if count > 0:

            with open('data/total_error_track.txt', 'a') as myfile:
                myfile.write('%.3f\n' % np.sum(error_track))
            print('Total error : %.3f' % np.sum(error_track))

            if plotting is True:

                plt.figure(1)
                plt.subplot(2, 1, 1)
                plt.plot(q_track)
                plt.legend(range(6))
                plt.title('Joint angles')

                plt.subplot(2, 1, 2)
                plt.plot(dq_track[10:])
                plt.legend(range(6))
                plt.title('Joint velocities')

                plt.figure(2)
                plt.plot(error_track)
                plt.ylabel('Euclidean distance to target (m)')
                plt.xlabel('Time steps')
                plt.title('Error over time')
                plt.savefig('data/error%i' % ii)

                plt.figure(3)
                ee_track = np.array(ee_track)
                targets_plot = np.ones(ee_track.shape) * target_xyz
                for jj in range(len(targets)):
                    targets_plot[jj] = targets[jj]
                abr_control.utils.plotting.plot_trajectory(ee_track, targets_plot)
                plt.show()

            print('Saving weights to file...')
            # save weights from adaptive population
            np.savez_compressed(
                'data/weights%i' % (ii % save_history),
                weights=ctrlr.sim.data[ctrlr.probe_weights])
            # save encoders from adaptive population
            np.savez_compressed(
                'data/encoders%i' % (ii % save_history),
                encoders=ctrlr.sim.data[ctrlr.probe_encoders])
            np.savez_compressed('data/ee%i' % ii, ee=ee_track)
