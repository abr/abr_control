"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1.5 simulated seconds, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np
import traceback

from abr_control.controllers import OSC
from abr_control.controllers import path_planners
from abr_control.interfaces import VREP
from abr_control.utils import DataHandler
import timeit
dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')

save_name = 'jaco_sim_no_weight_1'

# initialize our robot config
from abr_control.arms import jaco2 as arm
robot_config = arm.Config(use_cython=True, hand_attached=True)
# from abr_control.arms import ur5 as arm
# robot_config = arm.Config(use_cython=True)
# if using the Jaco 2 arm with the hand attached, use the following instead:
# robot_config = arm.Config(use_cython=True, hand_attached=True)

# instantiate controller
ctrlr = OSC(robot_config, kp=30, kv=20, vmax=1.0)

# instantiate path planner
path = path_planners.SecondOrder(robot_config=robot_config,
        n_timesteps=3000, w=1e4, zeta=3, threshold=0.05)
path_dt = 0.004
sim_dt = 0.004
targets = dat.load(params=['autogen_targets'],
        save_location='1lb_random_target')['autogen_targets']

for ii, target_xyz in enumerate(targets):
    print('Run %i, target: '%ii, target_xyz)
    # create our VREP interface
    interface = VREP(robot_config, dt=sim_dt)
    interface.connect()

    # set up lists for tracking data
    ee_track = []
    target_track = []
    u_osc = []
    filter_track = []
    time_track = []
    error_track = []
    q_track = []
    try:
        feedback = interface.get_feedback()
        # move arm to starting position
        interface.send_target_angles(q=robot_config.START_POS)
        # get the end-effector's initial position
        feedback = interface.get_feedback()
        start = robot_config.Tx('EE', feedback['q'])
        # make the target offset from that start position
        interface.set_xyz(name='target', xyz=target_xyz)

        # run ctrl.generate once to load all functions
        zeros = np.zeros(robot_config.N_JOINTS)
        ctrlr.generate(q=zeros, dq=zeros, target_pos=target_xyz)
        robot_config.orientation('EE', q=zeros)

        # calculate end-effector position
        ee_xyz = robot_config.Tx('EE', q=feedback['q'])
        # last three terms used as started point for target EE velocity
        target = np.concatenate((ee_xyz, np.array([0, 0, 0])), axis=0)
        print('\nSimulation starting...\n')

        run_time = 0.0
        while run_time < 40:
            start = interface.get_sim_time()
            #start = timeit.default_timer()
            # get joint angle and velocity feedback
            feedback = interface.get_feedback()
            target = path.step(state=target, target_pos=target_xyz,
                    dt=path_dt)

            # set position of filtered target
            interface.set_xyz(name='filtered_target', xyz=target[:3])

            # calculate the control signal
            u = ctrlr.generate(
                q=feedback['q'],
                dq=feedback['dq'],
                target_pos=target[:3],
                target_vel=target[3:])

            # get loop time before sim
            #time1 = timeit.default_timer()-start

            # send forces into VREP, step the sim forward
            interface.send_forces(u)

            current_joint_calc = robot_config.Tx('joint4', q=feedback['q'])
            print(current_joint_calc)
            # get loop time after sim
            #start = timeit.default_timer()

            # calculate end-effector position
            ee_xyz = robot_config.Tx('EE', q=feedback['q'])

            # caclulate error to target
            error = np.sqrt(np.sum((ee_xyz - target_xyz)**2))

            #loop_time = timeit.default_timer()-start + time1 + sim_dt
            loop_time = interface.get_sim_time() - start
            # track data
            ee_track.append(np.copy(ee_xyz))
            target_track.append(np.copy(target_xyz))
            u_osc.append(np.copy(u))
            filter_track.append(np.copy(target))
            time_track.append(loop_time)
            error_track.append(np.copy(error))
            q_track.append(np.copy(feedback['q']))

            run_time += loop_time
            if run_time %1 == 0:
                print('run time: ', run_time)
                print('dt: ', loop_time)

    except:
        print(traceback.format_exc())

    finally:
        # stop and reset the VREP simulation
        interface.disconnect()

        print('Simulation terminated...')

        overwrite = True
        create = True
        loc = 'simulations/%s/session000/run%03d'%(save_name, ii)

        custom_params = {'sim_dt': sim_dt, 'path_dt': path_dt}

        params = {'u_osc': u_osc, 'ee_xyz': ee_track, 'target': target_track,
                'filter': filter_track, 'time': time_track, 'error': error_track,
                'q': q_track}
        dat.save(data=params,
                save_location=loc, overwrite=overwrite, create=create)

        # Save OSC parameters
        dat.save(data=ctrlr.params,
                save_location=loc + ctrlr.params['source'], overwrite=overwrite, create=create)

        # # Save robot_config parameters
        # dat.save(data=robot_config.params,
        #         save_location=loc + robot_config.params['source'], overwrite=overwrite, create=create)

        # Save path planner parameters
        dat.save(data=path.params,
                save_location=loc + path.params['source'], overwrite=overwrite, create=create)

        if custom_params is not None:
            dat.save(data=custom_params,
                    save_location=loc + 'test_parameters', overwrite=overwrite,
                    create=create)
