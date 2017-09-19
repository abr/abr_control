"""
Move the jaco2 to a target position with an adaptive controller
that will account for a 2lb weight in its hand
"""
import numpy as np
import os
import timeit
import traceback
import redis

from abr_control.controllers import OSC, signals, path_planners
import abr_jaco2

class Training:

    def __init__(self):
        pass

    def run_test(self, time_scale=1, test_name="adaptive_training", trial=None, run=None,
             weights_file=None ,pes_learning_rate=1e-6, backend=None,
             autoload=False, time_limit=30, vision_target=False, offset=None):
        #TODO: Add name of paper once complete
        #TODO: Do we want to include nengo_spinnaker install instructions?
        #TODO: Add option to use our saved results incase user doesn't have
        # spinnaker
        """
        The test script used to collect data for training. The use of the
        adaptive controller and the type of backend can be specified. The script is
        also automated to use the correct weights files for continuous learning.
        The standard naming convention used is runs are consecutive tests where the previous
        learned weights are used. The whole of these runs are a single trial.
        Multiple trials of these runs can then be used for averaging and
        statictical purposes.

        The adaptive controller uses a 6 dimensional input and returns a 3 dimensional
        output. The base, shoulder and elbow joints' positions and velocities are
        used as the input, and the control signal for the corresponding joints gets
        returned.

        Parameters
        ----------
        time_scale: int, Optional (Default: 1)
            used for scaling the spinnaker input. Due to precision limit, spinnaker
            input gets scaled to allow for more decimals, then scaled back once
            extracted. Also can be used to account for learning speed since
            spinnaker runs in real time
        test_name: string, Optional (Default: "dewolf_2017_data")
            folder name where data is saved
        trial: int, Optional (Default: None)
            The current trial number, if left to None then it will be automatically
            updated based on the current trial. If the next trial is desired then
            this will need to be updated.
        run: int, Optional (Default: None)
            The current nth run that specifies to use the weights from the n-1 run.
            If set to None if will automatically increment based off the last saved
            weights.
        weights_file: string, Optional (Default: None)
            the path to the desired saved weights to use. If None will
            automatically take the most recent weights saved in the 'test_name'
            directory
        pes_learning_rate: float, Optional (Default: 1e-6)
            the learning rate for the adaptive population
        backend: string
            None: non adaptive control, Optional (Default: None)
            'nengo': use nengo as the backend for the adaptive population
            'spinnaker': use spinnaker as the adaptive population
        autoload: boolean, Optional (Default: False)
            True: used the specified weights, or the last set of learned weights if
                  not specified
            False: use the specified weights, if not specified start learning from
                   zero
        time_limit: float, Optional (Default: 30)
            the time limit for each run in seconds
        vision_target: boolean, Optional (Default: False)
            True: check redis server for targets sent from vision
            False: use hardcoded target locations
        offset: float array, Optional (Default: None)
            Set the offset to the end effector if something other than the default
            is desired. Use the form [x_offset, y_offset, z_offset]
        """

        # try to setup redis server if vision targets are desired
        if vision_target:
            try:
                import redis
                redis_server = redis.StrictRedis(host='localhost')
                self.redis_server = redis_server
            except ImportError:
                print('ERROR: Install redis to use vision targets, using preset targets')
                vision_target = False

        PRESET_TARGET = np.array([[.03, -.57, .87]])

        # initialize our robot config
        robot_config = abr_jaco2.Config(
            use_cython=True, hand_attached=True)
        self.robot_config = robot_config

        zeros = np.zeros(robot_config.N_JOINTS)

        # get Jacobians to each link for calculating perturbation
        J_links = [robot_config._calc_J('link%s' % ii, x=[0, 0, 0])
                   for ii in range(robot_config.N_LINKS)]

        # account for wrist to fingers offset
        R_func = robot_config._calc_R('EE')

        # Use user defined offset if one is specified
        if offset is None:
            OFFSET = robot_config.OFFSET
        else:
            OFFSET = offset

        robot_config.Tx('EE', q=zeros, x=OFFSET)

        # instantiate controller and path planner
        ctrlr = OSC(robot_config, kp=20, kv=6, vmax=1, null_control=True)
        path = path_planners.SecondOrder(robot_config)
        n_timesteps = 4000
        w = 1e4/n_timesteps
        zeta = 2
        dt = 0.003

        # run controller once to generate functions / take care of overhead
        # outside of the main loop, because force mode auto-exits after 200ms
        ctrlr.generate(zeros, zeros, np.zeros(3), offset=OFFSET)

        interface = abr_jaco2.Interface(robot_config)

        # temporarily set backend to nengo if non adaptive if selected so we can
        # still use the weight saving function in dynamics_adaptation
        backend_save = backend
        if backend is None:
            backend = 'nengo'
        # create our adaptive controller
        adapt = signals.DynamicsAdaptation(
            n_input=5,
            n_output=3,
            n_neurons=10000,
            pes_learning_rate=pes_learning_rate,
            intercepts=(-0.1, 1.0),
            weights_file=weights_file,
            backend=backend,
            trial=trial,
            run=run,
            test_name=test_name,
            autoload=autoload)
        backend = backend_save

        # connect to and initialize the arm
        interface.connect()
        interface.init_position_mode()
        interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

        # set up lists for tracking data
        time_track = []
        q_track = []
        u_track = []
        adapt_track = []
        error_track = []
        training_track = []
        target_track = []
        try:
            interface.init_force_mode()
            for ii in range(0,len(PRESET_TARGET)):
                # get the end-effector's initial position
                feedback = interface.get_feedback()

                # counter for print statements
                count = 0

                # track loop_time for stopping test
                loop_time = 0

                # get joint angle and velocity feedback
                feedback = interface.get_feedback()
                q = feedback['q']
                dq = feedback['dq']

                # calculate end-effector position
                ee_xyz = robot_config.Tx('EE', q=q, x= OFFSET)

                # last three terms used as started point for target velocity of
                # base 3 joints
                target = np.concatenate((ee_xyz, np.array([0, 0, 0])), axis=0)

                while loop_time < time_limit:
                    if vision_target is False:
                        TARGET_XYZ = PRESET_TARGET[ii]
                    else:
                        TARGET_XYZ = self.get_target_from_camera()
                        TARGET_XYZ = self.normalize_target(TARGET_XYZ)

                    start = timeit.default_timer()
                    prev_xyz = ee_xyz
                    target = path.step(y=target[:3], dy=target[3:], target=TARGET_XYZ, w=w,
                                       zeta=zeta, dt=dt)

                    # get joint angle and velocity feedback
                    feedback = interface.get_feedback()
                    q = feedback['q']
                    dq = feedback['dq']

                    # calculate end-effector position
                    ee_xyz = robot_config.Tx('EE', q=q, x= OFFSET)

                    # calculate the control signal
                    u_base = ctrlr.generate(
                        q=q,
                        dq=dq ,
                        target_pos=target[:3],
                        target_vel=target[3:],
                        offset = OFFSET)

                    # account for stiction in jaco2 base
                    if u_base[0] > 0:
                        u_base[0] *= 3.0
                    else:
                        u_base[0] *= 2.0
                    training_signal = np.array([ctrlr.training_signal[1],
                                                ctrlr.training_signal[2],
                                                ctrlr.training_signal[4]])

                    if backend != None:
                        # calculate the adaptive control signal
                        u_adapt = adapt.generate(
                            input_signal=np.array([robot_config.scaledown('q',q)[1],
                                                   robot_config.scaledown('q',q)[2],
                                                   robot_config.scaledown('q',q)[4],
                                                   robot_config.scaledown('dq',dq)[1],
                                                   robot_config.scaledown('dq',dq)[2]]),
                                    training_signal=training_signal)
                    else:
                        u_adapt = np.zeros(4)

                    # add adaptive signal to base controller
                    # u = u_base  + np.concatenate(((u_adapt / time_scale),
                    #                              np.array([0,0,0])), axis=0)
                    u_adapt = np.array([0,
                                        u_adapt[0]/time_scale,
                                        u_adapt[1]/time_scale,
                                        0,
                                        (u_adapt[2]/time_scale)/2,
                                        0])
                    u = u_base + u_adapt


                    # send forces
                    interface.send_forces(np.array(u, dtype='float32'))

                    # calculate euclidean distance to target
                    error = np.sqrt(np.sum((ee_xyz - TARGET_XYZ)**2))

                    # track data
                    q_track.append(np.copy(q))
                    u_track.append(np.copy(u))
                    adapt_track.append(np.copy(u_adapt))
                    error_track.append(np.copy(error))
                    training_track.append(np.copy(training_signal))
                    end = timeit.default_timer() - start
                    loop_time += end
                    time_track.append(np.copy(end))
                    target_track.append(np.copy(TARGET_XYZ))

                    if count % 1000 == 0:
                        print('error: ', error)
                        print('adapt: ', u_adapt/time_scale)
                        #print('hand: ', ee_xyz)
                        #print('target: ', target)
                        #print('control: ', u_base)

                    count += 1

        except:
            print(traceback.format_exc())

        finally:
            # close the connection to the arm
            interface.init_position_mode()
            interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
            interface.disconnect()

            if backend != None:
                # Save the learned weights
                adapt.save_weights(test_name=test_name, trial=trial, run=run)

            # get save location of weights to save tracked data in same directory
            [location, run_num] = adapt.weights_location(test_name=test_name, run=run,
                                                         trial=trial)
            if run_num == -1:
                run_num = 0
                print("RUN NUM IS -1 !!!!!!!!!!!!!!!!!!")

            print('Average loop speed: ', sum(time_track)/len(time_track))
            print('Run number ', run_num)
            print('Saving tracked data to ', location + '/run%i_data' % (run_num))

            time_track = np.array(time_track)
            q_track = np.array(q_track)
            u_track = np.array(u_track)
            adapt_track = np.array(adapt_track)
            error_track = np.array(error_track)
            training_track = np.array(training_track)

            if not os.path.exists(location + '/run%i_data' % (run_num)):
                os.makedirs(location + '/run%i_data' % (run_num))

            np.savez_compressed(location + '/run%i_data/q%i' % (run_num, run_num),
                                q=[q_track])
            np.savez_compressed(location + '/run%i_data/time%i' % (run_num, run_num),
                                time=[time_track])
            np.savez_compressed(location + '/run%i_data/u%i' % (run_num, run_num),
                                u=[u_track])
            np.savez_compressed(location + '/run%i_data/adapt%i' % (run_num, run_num),
                                adapt=[adapt_track])
            np.savez_compressed(location + '/run%i_data/target%i' % (run_num, run_num),
                                target=[target_track])
            np.savez_compressed(location + '/run%i_data/error%i' % (run_num, run_num),
                                error=[error_track])
            np.savez_compressed(location + '/run%i_data/training%i' % (run_num, run_num),
                                training=[training_track])
    def get_target_from_camera(self):
        # read from server
        camera_xyz = self.redis_server.get('target_xyz').decode('ascii')
        # if the target has changed, recalculate things
        camera_xyz = np.array([float(val) for val in camera_xyz.split()])
        # transform from camera to robot reference frame
        target_xyz = self.robot_config.Tx(
            'camera', x=camera_xyz, q=np.zeros(6))

        self.redis_server.set(
            'target_xyz_robot_coords', '%.3f %.3f %.3f' % tuple(target_xyz))

        return target_xyz

    def normalize_target(self, target, magnitude=0.9):
        # set it so that target is not too far from joint 1
        joint1_offset = np.array([0, 0, 0.273])
        norm = np.linalg.norm(target - joint1_offset)
        if norm > magnitude:
            target = ((target - joint1_offset) / norm) * magnitude + joint1_offset
        return target


