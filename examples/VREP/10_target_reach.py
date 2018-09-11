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
dat = DataHandler(use_cache=True, db_name='jacoOSCdebug')

save_name = 'jaco_sim_2'
notes = "inv and pinv, rcond = 0.04"

# initialize our robot config
if 'jaco' in save_name:
    from abr_control.arms import jaco2 as arm
    robot_config = arm.Config(use_cython=True, hand_attached=True)
elif 'ur5' in save_name:
    from abr_control.arms import ur5 as arm
    robot_config = arm.Config(use_cython=True)

# instantiate controller
ctrlr = OSC(robot_config, kp=30, kv=20, vmax=1.0, null_control=False)

# instantiate path planner
path = path_planners.SecondOrder(robot_config=robot_config,
        n_timesteps=3000, w=1e4, zeta=3, threshold=0.05)
path_dt = 0.004
sim_dt = 0.004
# targets = dat.load(params=['autogen_targets'],
#         save_location='1lb_random_target')['autogen_targets']
targets = np.array([
    [-0.09886674, 0.31435531, 0.69859133],
    [ 0.47701451, 0.06890294, 0.62898781],
    [ 0.33978046, -0.22197022, 0.69237045],
    [ 0.18900816, 0.37003439, 0.66534457],
    [-0.29493157, -0.3118809,  0.72051759],
    [-0.016, -0.4, 0.55],
    [ 0.018233, 0.02109056, 0.80556351],
    [-0.26065214, -0.04684662, 0.74356301],
    [ 0.52172849, -0.19677514, 0.6],
    [-0.47514801,  0.14137595, 0.55877516],
    ])

ref_frame = 'EE'

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
    Tx_track = []
    vmax_track = []
    u_Mx = []
    u_inertia = []
    u_g = []
    M = []
    M_inv = []
    Mx_inv = []
    Mx = []
    ref_frame_track = []
    ctrlr_q = []
    Mx_non_singular = []

    try:
        feedback = interface.get_feedback()
        # move arm to starting position
        interface.send_target_angles(q=robot_config.START_POS)
        # get the end-effector's initial position
        feedback = interface.get_feedback()
        start = robot_config.Tx(ref_frame, feedback['q'])
        # make the target offset from that start position
        interface.set_xyz(name='target', xyz=target_xyz)

        # run ctrl.generate once to load all functions
        zeros = np.zeros(robot_config.N_JOINTS)
        ctrlr.generate(q=zeros, dq=zeros, target_pos=target_xyz)
        robot_config.orientation(ref_frame, q=zeros)

        # calculate end-effector position
        ee_xyz = robot_config.Tx(ref_frame, q=feedback['q'],
                x=robot_config.OFFSET)
        # last three terms used as started point for target EE velocity
        target = np.concatenate((ee_xyz, np.array([0, 0, 0])), axis=0)
        print('\nSimulation starting...\n')

        run_time = 0.0
        while run_time < 4:
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
                target_vel=target[3:],
                ref_frame=ref_frame,
                offset=robot_config.OFFSET)

            # get loop time before sim
            #time1 = timeit.default_timer()-start

            # send forces into VREP, step the sim forward
            interface.send_forces(u)

            # get loop time after sim
            #start = timeit.default_timer()

            # calculate end-effector position
            ee_xyz = robot_config.Tx(ref_frame, q=feedback['q'],
                    x=robot_config.OFFSET)

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

            Tx_track.append(np.copy(ctrlr.Tx))
            vmax_track.append(np.copy(ctrlr.u_vmax))
            u_Mx.append(np.copy(ctrlr.u_Mx))
            u_inertia.append(np.copy(ctrlr.u_inertia))
            u_g.append(np.copy(ctrlr.u_g))
            M.append(np.copy(ctrlr.M))
            M_inv.append(np.copy(ctrlr.M_inv))
            Mx_inv.append(np.copy(ctrlr.Mx_inv))
            Mx.append(np.copy(ctrlr.Mx))
            ref_frame_track.append(np.copy(ctrlr.ref_frame))
            ctrlr_q.append(np.copy(ctrlr.q))
            Mx_non_singular.append(np.copy(ctrlr.Mx_non_singular))

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

        Mx_non_singular = [x for x in Mx_non_singular if x is not None]
        np.savez_compressed('Mx_non_singular',Mx_non_singular=Mx_non_singular)
        overwrite = True
        create = True
        loc = 'simulations/%s/session000/run%03d'%(save_name, ii)

        custom_params = {'sim_dt': sim_dt, 'path_dt': path_dt, 'Notes': notes}

        params = {'u_base': u_osc, 'ee_xyz': ee_track, 'target': target_track,
                'filter': filter_track, 'time': time_track, 'error': error_track,
                'q': q_track, 'Tx': Tx_track, 'u_vmax': vmax_track,
                'u_Mx': u_Mx, 'u_inertia': u_inertia, 'u_g': u_g, 'M': M, 'M_inv':
                M_inv, 'Mx_inv': Mx_inv, 'Mx': Mx, 'ref_frame':
                ref_frame_track, 'ctrlr_q': ctrlr_q, 'Mx_non_singular':
                Mx_non_singular}
        dat.save(data=params,
                save_location=loc, overwrite=overwrite, create=create)

        # Save OSC parameters
        dat.save(data=ctrlr.params,
                    save_location='simulations/%s/parameters/%s'%(save_name,
                    ctrlr.params['source']), overwrite=overwrite, create=create)

        # Save path planner parameters
        dat.save(data=path.params,
                    save_location='simulations/%s/parameters/%s'%(save_name,
                    path.params['source']), overwrite=overwrite, create=create)

        if custom_params is not None:
            dat.save(data=custom_params,
                    save_location='simulations/%s/parameters/%s'%(save_name,
                    'test_parameters'), overwrite=overwrite,
                    create=create)
