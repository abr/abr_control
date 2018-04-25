"""
Move the jaco2 to a target position with an adaptive controller
that will account for a 2lb weight in its hand
"""
import numpy as np
import os
import timeit
import time
import traceback
import scipy
import redis

from abr_control.controllers import OSC, signals, path_planners
import abr_jaco2
import nengo

class Training:

    def __init__(self):
        pass

    def run_test(self, n_neurons=1000, n_ensembles=1, decimal_scale=1,
                 test_name="adaptive_training", session=None, run=None,
                 weights_file=None ,pes_learning_rate=1e-6, backend=None,
                 autoload=False, time_limit=30, target_type='single',
                 offset=None, avoid_limits=False, additional_mass=0,
                 kp=20, kv=6, ki=0, integrate_err=False, ee_adaptation=False,
                 joint_adaptation=False, simulate_wear=False,
                 probe_weights=False, seed=None, friction_bootstrap=False,
                 redis_adaptation=False, SCALES=None, MEANS=None,
                 gradual_wear=False, print_run=None):
        #TODO: DElete print_run  parameter
        """
        The test script used to collect data for training. The use of the
        adaptive controller and the type of backend can be specified. The script is
        also automated to use the correct weights files for continuous learning.
        The standard naming convention used is runs are consecutive tests where the previous
        learned weights are used. The whole of these runs are a single session.
        Multiple sessions of these runs can then be used for averaging and
        statictical purposes.

        The adaptive controller uses a 6 dimensional input and returns a 3 dimensional
        output. The base, shoulder and elbow joints' positions and velocities are
        used as the input, and the control signal for the corresponding joints gets
        returned.

        Parameters
        ----------
        n_neurons: int, Optional (Default: 1000)
            the number of neurons in the adaptive population
        n_ensembles: int, Optional (Default: 1)
            the number of ensembles of n_neurons number of neurons
        decimal_scale: int, Optional (Default: 1)
            used for scaling the spinnaker input. Due to precision limit, spinnaker
            input gets scaled to allow for more decimals, then scaled back once
            extracted. Also can be used to account for learning speed since
            spinnaker runs in real time
        test_name: string, Optional (Default: "dewolf_2017_data")
            folder name where data is saved
        session: int, Optional (Default: None)
            The current session number, if left to None then it will be automatically
            updated based on the current session. If the next session is desired then
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
        target_type: string, Optional (Default: single)
            single: one target
            multi: five targets
            figure8: move in figure8 pattern
            vision: get target from vision system
        offset: float array, Optional (Default: None)
            Set the offset to the end effector if something other than the default
            is desired. Use the form [x_offset, y_offset, z_offset]
        avoid_limits: boolean, Optional (Default: False)
            set true if there are limits you would like to avoid
        additional_mass: float, Optional (Default: 0)
            any extra mass added to the EE if known [kg]
        kp: float, Optional (Default: 20)
            proportional gain term
        kv: float, Optional (Default: 6)
            derivative gain term
        ki: float, Optional (Default: 0)
            integral gain term
        integrate_err: Boolean, Optional (Default: False)
            True to add I term to PD control (PD -> PID)
        ee_adaptation: Booleanm Optional (Default: False)
            True if doing neural adaptation in task space
        joint_adaptation: Boolean Optional (Default: False)
            True if doing neural adaptation in joint space
        simulate_wear: Boolean Optional (Default: False)
            True to add a simulated wearing of the joints that mimics friction
            by opposing motion
        probe_weights: Boolean Optional (Default: False)
            True to probe adaptive population for decoders
        seed: int Optional, (Default: None)
            seed used for random number generation in adaptive population
        friction_bootstrap: Boolean Optional (Default: False)
            True if want to pass an estimate of the friction signal as starting
            point for adaptive population
        redis_adaptation: Boolean Optional (Default: False)
            True to send adaptive inputs to redis and read outputs back, allows
            user to use any backend they like as long as it can read/write from
            redis
        gradual_wear: Boolean Optional (Default: False)
            sets a gain on the friction term
            False: 1
            True: linear gradient based on run number

        Attributes
        ----------
        TARGET_XYZ: array of floats
            the goal target, used for calculating error
        target: array of floats [x, y, z, vx, vy, vz]
            the filtered target, used for calculating error for figure8
            test since following trajectory and not moving to a single final
            point
        """
        print("RUN PASSED IN IS: ", run)

        if redis_adaptation:
            try:
                import nengolib
                rng = np.random.RandomState(seed)

                intercepts = AreaIntercepts(
                    dimensions=4,
                    base=Triangular(-0.9, -0.9, 0.0))
                intercepts = intercepts.sample(n_neurons, rng=rng)
                intercepts = np.array(intercepts)

                encoder_dist = (nengolib.stats.ScatteredHypersphere(surface=True))
                encoders = encoder_dist.sample(n_neurons, d=4, rng=rng)
                # encoders = nengo.dists.Choice([[1],[-1]]).sample(n_neurons,
                #         d=4, rng=rng)
                encoders = np.array(encoders)
                # need to set this so that the ellipses separating the large
                # array don't get passed in the string
                np.set_printoptions(threshold=np.prod(encoders.shape))
                print('NengoLib used to optimize encoders placement')

            except ImportError:
                print('You must either install nengolib, or manually pass in'
                       + ' encoders')
            try:
                import redis
                redis_server = redis.StrictRedis(host='127.0.0.1', port=6379, db=0)

                # Initialize parameters in redis
                redis_server.set('n_neurons', '%d'%n_neurons)
                redis_server.set('dimensions','%d'%4)
                redis_server.set('preTraceSLI', '%d'%0)
                redis_server.set('preTraceSLF', '%d'%8)
                redis_server.set('error_scale', '%.3f' % 6)
                redis_server.set('bias_exp', '%d' %7)
                redis_server.set('encoder_exp', '%d' %0)
                redis_server.set('decoder_exp', '%d' %0)
                redis_server.set('output_spiking', 'False')
                redis_server.set('get_hidden_layer_spikes', 'False')
                redis_server.set('tau_rc', '%.3f'%0.02)#'np.inf')
                redis_server.set('tau_ref','%.3f'%0.002)#0.001)
                redis_server.set('learn_tau', '%.3f'%0.01)
                redis_server.set('input_synapse','%.3f'%0.004)#0.01)
                redis_server.set('output_synapse','%.3f'%0.002)#0.01)
                redis_server.set('encoders',
                        np.array2string(encoders, separator=','))
                redis_server.set('biases','None')
                redis_server.set('gains','None')
                redis_server.set('max_rates','%d %d'%(200,400))
                redis_server.set('intercepts',
                        np.array2string(intercepts)[2:-1])
                redis_server.set('dt','%.3f'%0.001)
                redis_server.set('neuron_threshold','%d'%600)#2000)
                redis_server.set('output_threshold','%d'%1000)#2000)
                redis_server.set('decoder_scale','%d'%4)#8)
                redis_server.set('max_input_rate','%d'%1000)#300)
                redis_server.set('max_output_rate','%d'%300)#300)
                redis_server.set('output_dims','%d'%2)
                redis_server.set('max_neurons_per_core','%d'%1004)
                redis_server.set('debug', 'False')
                redis_server.set('load_weights', 'False')
                redis_server.set('save_weights', 'True')
                redis_server.set('data_saved', 'False')
                redis_server.set('u_adapt', '%.3f %.3f'%(0,0))
                redis_server.set('run_test', 'False')
                redis_server.set('chip_ready', 'False')
                redis_server.set('time_limit', '%d'%time_limit)
                redis_server.set('seed', '%d'%seed)
                redis_server.set('input_signal', '%.3f %.3f %.3f %.3f' %
                        (0,0,0,0))
                redis_server.set('training_signal', '%.3f %.3f' %(0, 0))

            except ImportError:
                print("You must install redis to do adaptation through a redis"
                      + " server")

        # Set the target based on the source and type of test
        PRESET_TARGET = np.array([[.57, 0.03, .87]])

        if target_type == 'multi':
            PRESET_TARGET = np.array([[.56, -.09, .95],
                                      [.12, .15, .80],
                                      [.80, .26, .61],
                                      [.38, .46, .81],
                                      [.0, .0, 1.15]])
            time_limit /= len(PRESET_TARGET)

        elif target_type == 'vision':
            # try to setup redis server if vision targets are desired
            try:
                import redis
                redis_server = redis.StrictRedis(host='localhost')
                self.redis_server = redis_server
            except ImportError:
                print('ERROR: Install redis to use vision targets, using preset targets')


        elif target_type == 'figure8':
            try:
                import pydmps
            except ImportError:
                print('\npydmps library required, see ' +
                      'https://github.com/studywolf/pydmps\n')

            # create a dmp that traces a figure8
            x = np.linspace(0, np.pi*2, 100)
            dmps_traj = np.array([0.2*np.sin(x), 0.2*np.sin(2*x)+0.7])
            dmps = pydmps.DMPs_rhythmic(n_dmps=2, n_bfs=50, dt=0.001)
            dmps.imitate_path(dmps_traj)

        # get averages and scale of target locations for scaling into network
        x_avg = np.mean(PRESET_TARGET[:,0])
        y_avg = np.mean(PRESET_TARGET[:,1])
        z_avg = np.mean(PRESET_TARGET[:,2])

        x_scale = np.max(PRESET_TARGET[:,0]) - np.min(PRESET_TARGET[:,0])
        y_scale = np.max(PRESET_TARGET[:,1]) - np.min(PRESET_TARGET[:,1])
        z_scale = np.max(PRESET_TARGET[:,2]) - np.min(PRESET_TARGET[:,2])

        # initialize our robot config
        robot_config = abr_jaco2.Config(
            use_cython=True, hand_attached=True, SCALES=SCALES, MEANS=MEANS)
        self.robot_config = robot_config

        zeros = np.zeros(robot_config.N_JOINTS)

        # get Jacobians to each link for calculating perturbation
        self.J_links = [robot_config._calc_J('link%s' % ii, x=[0, 0, 0])
                   for ii in range(robot_config.N_LINKS)]

        self.JEE  = robot_config._calc_J('EE', x=[0, 0, 0])

        # account for wrist to fingers offset
        R_func = robot_config._calc_R('EE')

        # Use user defined offset if one is specified
        if offset is None:
            OFFSET = robot_config.OFFSET
        else:
            OFFSET = offset

        robot_config.Tx('EE', q=zeros, x=OFFSET)

        # temporarily set backend to nengo if non adaptive if selected so we can
        # still use the weight saving function in dynamics_adaptation.py in the
        # abr_control repo
        if backend is None:
            adapt_backend = 'nengo'
        else:
            adapt_backend = backend

        # create our adaptive controller
        if ee_adaptation and joint_adaptation:
            n_input = 4
            n_output = 3
        elif ee_adaptation and not joint_adaptation:
            n_input = 3
            n_output = 3
            pes_learning_rate /= ki
        elif joint_adaptation and not ee_adaptation:
            n_input = 4
            n_output = 2
        elif redis_adaptation:
            n_input=4
            n_output=2
        else:
            n_input = 4
            n_output = 2

        # for use with weight estimation, is the number of joint angles being
        # passed to the adaptive controller (not including velocities)
        self.adapt_dim = n_output
        self.decimal_scale = decimal_scale

        # if user specifies an additional mass, use the estimation function to
        # give the adaptive controller a better starting point
        self.additional_mass = additional_mass
        get_run = signals.DynamicsAdaptation(n_input=1, n_output=1)
        if redis_adaptation:
            # pass parameters to redis
            # get save location of saved data
            [location, run_num] = get_run.weights_location(test_name=test_name, run=run,
                                                         session=session)
            print('run NUM: ', run_num)
            print('loc', location)
            if run_num == 0:
                # first run of adaptation, no learned parameters to pass in so
                # keep defaults from initialization
                print('First run of learning, using default parameters')
            else:
                print('Loading weights from %s run%i'%(location, run_num-1))

                # with open('%s/run%i/encoders%i.bin'%
                #         (location, run_num-1, run_num-1), 'rb') as f:
                #     encoders = f.read()
                with open('%s/run%i_data/weights%i.bin'%
                        (location, run_num-1, run_num-1), 'rb') as f:
                    weights = f.read()
                # with open('%s/run%i/biases%i.bin'%
                #         (location, run_num-1, run_num-1), 'rb') as f:
                #     biases = f.read()

                # encoders = np.load('%s/run%i/encoders%i.npz'%(location,
                #     run_num-1, run_num-1))['encoders']
                # weights = np.load('%s/run%i_data/weights%i.npz'%(location,
                #     run_num-1, run_num-1))['weights']
                # biases = np.load('%s/run%i/biases%i.npz'%(location,
                #     run_num-1, run_num-1))['biases']

                #redis_server.set('encoders', encoders)
                redis_server.set('weights', weights)
                #redis_server.set('biases', biases)
                redis_server.set('load_weights', 'True')

            redis_server.set('run_test', 'True')

        elif self.additional_mass != 0 and weights_file is None:
            # if the user knows about the mass at the EE, try and improve
            # our starting point
            self.fake_gravity = np.array([[0, 0, -9.81*self.additional_mass, 0, 0, 0]]).T
            print('Using mass estimate of %f kg as starting point'
                  %self.additional_mass)
            adapt = signals.DynamicsAdaptation(
                n_input=n_input,
                n_output=n_output,
                n_neurons=n_neurons,
                n_ensembles=n_ensembles,
                pes_learning_rate=pes_learning_rate,
                intercepts=(-0.5, -0.2),
                weights_file=weights_file,
                backend=adapt_backend,
                session=session,
                run=run,
                test_name=test_name,
                autoload=autoload,
                function=self.gravity_estimate,
                probe_weights=probe_weights,
                seed=seed)

        elif friction_bootstrap and weights_file is None:
            # if the user knows about the friction, try and improve
            # our starting point
            print('Using friction estimate as starting point')
            adapt = signals.DynamicsAdaptation(
                n_input=n_input,
                n_output=n_output,
                n_neurons=n_neurons,
                n_ensembles=n_ensembles,
                pes_learning_rate=pes_learning_rate,
                intercepts=(-0.5, -0.2),
                weights_file=weights_file,
                backend=adapt_backend,
                session=session,
                run=run,
                test_name=test_name,
                autoload=autoload,
                function=lambda x: self.friction(x) * -1,
                probe_weights=probe_weights,
                seed=seed)
        else:
            adapt = signals.DynamicsAdaptation(
                n_input=n_input,
                n_output=n_output,
                n_neurons=n_neurons,
                n_ensembles=n_ensembles,
                pes_learning_rate=pes_learning_rate,
                intercepts=(-0.5, -0.2),
                weights_file=weights_file,
                backend=adapt_backend,
                session=session,
                run=run,
                test_name=test_name,
                autoload=autoload,
                probe_weights=probe_weights,
                seed=seed)

        # if using integral term for PID control, check if previous weights
        # have been saved
        if integrate_err:
            [location, run_num] = get_run.weights_location(test_name=test_name, run=run,
                                                         session=session)
            if run_num < 1:
                run_num = 0
                int_err_prev = [0, 0, 0]
            else:
                int_err_prev = np.squeeze(np.load(location + '/run%i_data/int_err%i.npz'
                                       % (run_num-1, run_num-1))['int_err'])[-1]
        else:
            int_err_prev = None

        # instantiate operational space controller
        print('using int err of: ', int_err_prev)
        ctrlr = OSC(robot_config, kp=kp, kv=kv, ki=ki, vmax=1,
                    null_control=True, integrated_error=int_err_prev)

        # add joint avoidance controller if specified to do so
        if avoid_limits:
            avoid = signals.AvoidJointLimits(
                      robot_config,
                      # joint 4 flipped because does not cross 0-2pi line
                      min_joint_angles=[None, 1.1, 0.5, 3.5, 2.0, 1.6],
                      max_joint_angles=[None, 3.65, 6.25, 6.0, 5.0, 4.6],
                      max_torque=[4]*robot_config.N_JOINTS,
                      cross_zero=[True, False, False, False, True, False],
                      gradient = [False, True, True, True, True, False])
        # if not planning trajectory (figure8 target) then use a second
        # order filter to smooth out the trajectory to target(s)
        if target_type != 'figure8':
            path = path_planners.SecondOrder(robot_config, n_timesteps=4000,
                      w=1e4, zeta=2, threshold=0.05)
            dt = 0.003

        # run controller once to generate functions / take care of overhead
        # outside of the main loop, because force mode auto-exits after 200ms
        ctrlr.generate(zeros, zeros, np.zeros(3), offset=OFFSET)

        interface = abr_jaco2.Interface(robot_config)

        # connect to and initialize the arm
        interface.connect()
        interface.init_position_mode()
        interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

        # set up lists for tracking data
        time_track = []
        q_track = []
        dq_track = []
        u_track = []
        adapt_track = []
        error_track = []
        training_track = []
        target_track = []
        ee_track = []
        input_signal = []
        filter_track = []
        if integrate_err:
            int_err_track = []
        if simulate_wear:
            self.F_prev = np.array([0, 0])
            if gradual_wear:
                friction_gains = np.linspace(0,1,50)
                kfriction = friction_gains[print_run]
                print('run_:',run)
                print("Using kfriction: ", kfriction)
            friction_track = []

        if redis_adaptation:
            chip_ready = redis_server.get('chip_ready').decode('ascii')
            print("Waiting for chip to start test...")
            while chip_ready == 'False':
                # wait 5ms and check redis again to see if chip is ready
                time.sleep(0.005)
                chip_ready = redis_server.get('chip_ready').decode('ascii')
            print("Chip ready, starting test")
        try:
            interface.init_force_mode()
            for ii in range(0,len(PRESET_TARGET)):
                # counter for print statements
                count = 0

                # track loop_time for stopping test
                loop_time = 0

                # get joint angle and velocity feedback
                feedback = interface.get_feedback()
                q = feedback['q']
                dq = feedback['dq']
                dq[abs(dq) < 0.05] = 0
                # calculate end-effector position
                ee_xyz = robot_config.Tx('EE', q=q, x= OFFSET)

                # last three terms used as started point for target EE velocity
                target = np.concatenate((ee_xyz, np.array([0, 0, 0])), axis=0)

                # M A I N   C O N T R O L   L O O P
                while loop_time < time_limit:
                    start = timeit.default_timer()
                    prev_xyz = ee_xyz

                    if target_type == 'vision':
                        TARGET_XYZ = self.get_target_from_camera()
                        TARGET_XYZ = self.normalize_target(TARGET_XYZ)
                        target = path.step(state=target,
                                target_pos=TARGET_XYZ, dt=dt)

                    elif target_type == 'figure8':
                        y, dy, ddy = dmps.step()
                        target = [0.65, y[0], y[1], 0, dy[0], dy[1]]
                        TARGET_XYZ = target[:3]

                    else:
                        TARGET_XYZ = PRESET_TARGET[ii]
                        target = path.step(state=target, target_pos=TARGET_XYZ,
                                dt=dt)

                    # calculate euclidean distance to target
                    error = np.sqrt(np.sum((ee_xyz - TARGET_XYZ)**2))

                    # get joint angle and velocity feedback
                    feedback = interface.get_feedback()
                    q = feedback['q']
                    dq = feedback['dq']
                    dq[abs(dq) < 0.05] = 0

                    # calculate end-effector position
                    ee_xyz = robot_config.Tx('EE', q=q, x= OFFSET)

                    if ee_adaptation and joint_adaptation:
                        # adaptive control in state space
                        training_signal = TARGET_XYZ - ee_xyz
                        # calculate the adaptive control signal
                        adapt_input = np.array([robot_config.scaledown('q',q)[1],
                                                   robot_config.scaledown('q',q)[2],
                                                   robot_config.scaledown('dq',dq)[1],
                                                   robot_config.scaledown('dq',dq)[2]])
                        u_adapt = adapt.generate(input_signal=adapt_input,
                                                 training_signal=training_signal)
                        u_adapt /= decimal_scale

                    elif ee_adaptation and not joint_adaptation:
                        # adaptive control in state space
                        training_signal = TARGET_XYZ - ee_xyz
                        # scale inputs
                        adapt_input = np.array([(ee_xyz[0] - x_avg) / x_scale,
                                                (ee_xyz[1] - y_avg) / y_scale,
                                                (ee_xyz[2] - z_avg) / z_scale])

                        u_adapt = adapt.generate(input_signal=adapt_input,
                                                 training_signal=training_signal)
                        u_adapt /= decimal_scale

                    else:
                        u_adapt = None

                    # Need to create controller here, in case using ee
                    # adaptation, it needs to be passed to the OSC controller
                    # calculate the base operation space control signal
                    u_base = ctrlr.generate(
                        q=q,
                        dq=dq ,
                        target_pos=target[:3],
                        target_vel=target[3:],
                        offset = OFFSET)#,
                        #ee_adapt=u_adapt)

                    # account for stiction in jaco2 base
                    if u_base[0] > 0:
                        u_base[0] *= 3.0
                    else:
                        u_base[0] *= 2.0

                    if joint_adaptation and not ee_adaptation:
                        training_signal = np.array([ctrlr.training_signal[1],
                                                    ctrlr.training_signal[2]])
                        # calculate the adaptive control signal
                        adapt_input = np.array([robot_config.scaledown('q',q)[1],
                                                   robot_config.scaledown('q',q)[2],
                                                   robot_config.scaledown('dq',dq)[1],
                                                   robot_config.scaledown('dq',dq)[2]])
                        u_adapt = adapt.generate(input_signal=adapt_input,
                                                 training_signal=training_signal)

                        # add adaptive signal to base controller
                        u_adapt = np.array([0,
                                            u_adapt[0]/decimal_scale,
                                            u_adapt[1]/decimal_scale,
                                            0,
                                            0,
                                            0])
                        u = u_base + u_adapt

                    elif redis_adaptation:
                        training_signal = np.array([ctrlr.training_signal[1],
                                                    ctrlr.training_signal[2]])
                        adapt_input = np.array([robot_config.scaledown('q',q)[1],
                                                   robot_config.scaledown('q',q)[2],
                                                   robot_config.scaledown('dq',dq)[1],
                                                   robot_config.scaledown('dq',dq)[2]])
                        # send input information to redis
                        # redis_server.set('input_signal', '%.3f %.3f %.3f %.3f' %
                        #                      (np.sin(loop_time),
                        #                       np.cos(loop_time),
                        #                       np.sin(np.sqrt(2) * loop_time),
                        #                       np.cos(np.sqrt(2) * loop_time)))
                        redis_server.set('input_signal', '%.3f %.3f %.3f %.3f' %
                                             (adapt_input[0], adapt_input[1],
                                              adapt_input[2], adapt_input[3]))
                        redis_server.set('training_signal', '%.3f %.3f' %
                                             (training_signal[0],training_signal[1]))
                        # get adaptive output back
                        u_adapt = redis_server.get('u_adapt').decode('ascii')
                        u_adapt = np.array([float(val) for val in u_adapt.split()])
                        # redis_server.set('training_signal', '%.3f %.3f' %
                        #                      (np.sin(loop_time)-u_adapt[0],
                        #                       np.cos(loop_time) - u_adapt[1]))
                        u_adapt = np.array([0,
                                            3*u_adapt[0]/20,
                                            3*u_adapt[1]/20,
                                            0,
                                            0,
                                            0])
                        u = u_base + u_adapt

                    else:
                        u = u_base
                        u_adapt = None
                        #training_signal = np.zeros(3)
                        training_signal = np.array([ctrlr.training_signal[1],
                                                    ctrlr.training_signal[2]])
                        adapt_input = np.zeros(3)


                    # add limit avoidance if True
                    if avoid_limits:
                        u += avoid.generate(q)

                    if simulate_wear:
                        u_friction = self.friction(dq[1:3])
                        if gradual_wear:
                            u_friction *= kfriction
                        u += [0, u_friction[0], u_friction[1], 0, 0, 0]
                        friction_track.append(np.copy(u_friction))

                    # send forces
                    interface.send_forces(np.array(u, dtype='float32'))

                    # track data
                    q_track.append(np.copy(q))
                    dq_track.append(np.copy(dq))
                    u_track.append(np.copy(u))
                    adapt_track.append(np.copy(u_adapt))
                    error_track.append(np.copy(error))
                    training_track.append(np.copy(training_signal))
                    end = timeit.default_timer() - start
                    loop_time += end
                    time_track.append(np.copy(end))
                    target_track.append(np.copy(TARGET_XYZ))
                    input_signal.append(np.copy(adapt_input))
                    ee_track.append(np.copy(ee_xyz))
                    filter_track.append(np.copy(target))
                    if integrate_err:
                        int_err_track.append(np.copy(ctrlr.integrated_error))

                    if count % 1000 == 0:
                        print('error: ', error)
                        print('dt: ', end)
                        print('adapt: ', u_adapt)
                        #print('int_err/ki: ', ctrlr.int_err)
                        #print('q: ', q)
                        #print('hand: ', ee_xyz)
                        #print('target: ', target)
                        #print('control: ', u_base)

                    count += 1

        except:
            print(traceback.format_exc())

        finally:

            if redis_adaptation:
                redis_server.set('run_test', 'False')

            # close the connection to the arm
            interface.init_position_mode()
            print('NUMBER OF LOOP STEPS: ', count)

            print("RUN IN IS: ", run)
            # get save location of weights to save tracked data in same directory
            [location, run_num] = get_run.weights_location(test_name=test_name, run=run,
                                                         session=session)
            print("RUN OUT IS: ", run_num)
            if backend != None:
                run_num += 1
                # Save the learned weights
                adapt.save_weights(test_name=test_name, session=session, run=run)

            interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
            interface.disconnect()

            if redis_adaptation:

                if not os.path.exists(location + '/run%i_data' % (run_num)):
                    os.makedirs(location + '/run%i_data' % (run_num))

                data_saved = redis_server.get('data_saved').decode('ascii')
                while data_saved == 'False':
                    # wait for flag that redis has been updated
                    time.sleep(0.05)
                    data_saved = redis_server.get('data_saved').decode('ascii')

                #encoders = redis_server.get('encoders')
                weights = redis_server.get('weights')
                # biases = redis_server.get('biases')
                # with open('%s/run%i/encoders%i.bin'%
                #         (location,run_num,run_num), 'wb') as f:
                #     f.write(encoders)
                with open('%s/run%i_data/weights%i.bin'%
                        (location,run_num,run_num), 'wb') as f:
                    f.write(weights)
                # with open('%s/run%i/biases%i.bin'%
                #         (location,run_num,run_num), 'wb') as f:
                #     f.write(biases)
                # np.savez_compressed('%s/run%i/encoders%i.npz'%(location,run_num,run_num),
                #         encoders=encoders)
                #np.savez_compressed('%s/run%i_data/weights%i.npz'%(location,run_num,run_num),
                #        weights=weights)
                # np.savez_compressed('%s/run%i/biases%i.npz'%(location,run_num,run_num),
                #         biases=biases)
                redis_server.set('data_saved', 'False')

            print('**** RUN STATS ****')
            print('Average loop speed: ', sum(time_track)/len(time_track))
            print('AVG Error/Step: ', sum(error_track)/len(error_track))
            print('Run number ', run_num)
            print('Saving tracked data to ', location + '/run%i_data' % (run_num))
            print('*******************')

            time_track = np.array(time_track)
            q_track = np.array(q_track)
            u_track = np.array(u_track)
            adapt_track = np.array(adapt_track)
            error_track = np.array(error_track)
            training_track = np.array(training_track)
            input_signal = np.array(input_signal)
            ee_track = np.array(ee_track)
            dq_track = np.array(dq_track)
            filter_track = np.array(filter_track)
            if integrate_err:
                int_err_track = np.array(int_err_track)
            if simulate_wear:
                friction_track = np.array(friction_track)

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
            np.savez_compressed(location + '/run%i_data/input_signal%i' % (run_num, run_num),
                                input_signal=[input_signal])
            np.savez_compressed(location + '/run%i_data/ee_xyz%i' % (run_num, run_num),
                                ee_xyz=[ee_track])
            np.savez_compressed(location + '/run%i_data/dq%i' % (run_num, run_num),
                                dq=[dq_track])
            np.savez_compressed(location + '/run%i_data/filter_track%i' % (run_num, run_num),
                                filter_track=[filter_track])
            if probe_weights:
                np.savez_compressed(location + '/run%i_data/probe%i' % (run_num, run_num),
                                    probe=[adapt.sim.data[adapt.nengo_model.weights_probe]])
            if integrate_err:
                np.savez_compressed(location + '/run%i_data/int_err%i' % (run_num, run_num),
                                    int_err=[int_err_track])
            if simulate_wear:
                np.savez_compressed(location + '/run%i_data/friction%i' % (run_num, run_num),
                                    friction=[friction_track])
                if gradual_wear:
                    np.savez_compressed(location + '/run%i_data/kfriction%i' % (run_num, run_num),
                                        kfriction=[kfriction])


            if backend != 'nengo_spinnaker':
                # give user time to pause before the next run starts, only
                # works if looping through tests using a bash script
                print('2 Seconds to pause: Hit ctrl z to pause, fg to resume')
                time.sleep(1)
                print('1 Second to pause')
                time.sleep(1)
                print('Starting next test...')

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
        # set it so that target is not too far from shoulder
        joint1_offset = np.array([0, 0, 0.273])
        norm = np.linalg.norm(target - joint1_offset)
        if norm > magnitude:
            target = ((target - joint1_offset) / norm) * magnitude + joint1_offset
        return target

    # TODO parameterize this so can use for different tests, more complex have
    # more dimensions, simple test only had q1 and q2
    def gravity_estimate(self, x):
        #q1, q2 = x[:self.adapt_dim]  # x = [q, dq]
        q0, q1, q2 = x[:self.adapt_dim]  # x = [q, dq]
        g_avg = []
        dist = nengo.dists.UniformHypersphere()
        samples = dist.sample(1000, d=self.robot_config.N_JOINTS - self.adapt_dim)
        for sample in samples:
            #q = np.hstack([sample[0], q0, q1, q2, sample[1:]])
            q = np.hstack([q0, q1, q2, sample])
            q = self.robot_config.scaleup('q', q)
            pars = tuple(q) + tuple([0, 0, 0])
            g = np.dot(self.JEE(*pars).T, self.fake_gravity)
            g_avg.append(g.squeeze())
        #g_avg = np.mean(np.array(g_avg), axis=0)[[1, 2]]
        g_avg = np.mean(np.array(g_avg), axis=0)[:3]
        return -g_avg*self.decimal_scale

    def friction(self, x):
        if len(x) > self.adapt_dim:
            # if entire adaptive input is passed in, only take the velocity
            # (second half)
            v = np.copy(x[self.adapt_dim:])
        else:
            # if x matches the length of adaptive dim then we have received
            # velocity directly
            v = np.copy(x)
        sgn_v = np.sign(v)
        # forces selected to have friction in a reasonable force range wrt the
        # forces the arm moves with (friction in range of 2-4N for typical
        # movement)
        Fn = 4
        # using static and kinetic coefficients of friction for steel on steel
        uk = 0.42
        us = 0.74
        vs = 0.1
        Fc = -uk * Fn * sgn_v
        Fs = -us * Fn * sgn_v
        Fv = 1.2
        Ff = Fc + (Fs-Fc) * np.exp(-sgn_v * v/vs) - Fv * v
        Ff *= np.array([0.9, 1.1])
        return(Ff)

class AreaIntercepts(nengo.dists.Distribution):
    """ Generate an optimally distributed set of intercepts in
    high-dimensional space.
    """
    dimensions = nengo.params.NumberParam('dimensions')
    base = nengo.dists.DistributionParam('base')

    def __init__(self, dimensions, base=nengo.dists.Uniform(-1, 1)):
        super(AreaIntercepts, self).__init__()
        self.dimensions = dimensions
        self.base = base

    def __repr(self):
        return ("AreaIntercepts(dimensions=%r, base=%r)" %
                (self.dimensions, self.base))

    def transform(self, x):
        sign = 1
        if x > 0:
            x = -x
            sign = -1
        return sign * np.sqrt(1 - scipy.special.betaincinv(
            (self.dimensions + 1) / 2.0, 0.5, x + 1))

    def sample(self, n, d=None, rng=np.random):
        s = self.base.sample(n=n, d=d, rng=rng)
        for i in range(len(s)):
            s[i] = self.transform(s[i])
        return s

class Triangular(nengo.dists.Distribution):
    """ Generate an optimally distributed set of intercepts in
    high-dimensional space using a triangular distribution.
    """
    left = nengo.params.NumberParam('dimensions')
    right = nengo.params.NumberParam('dimensions')
    mode = nengo.params.NumberParam('dimensions')

    def __init__(self, left, mode, right):
        super(Triangular, self).__init__()
        self.left = left
        self.right = right
        self.mode = mode

    def __repr__(self):
        return ("Triangular(left=%r, mode=%r, right=%r)" %
                (self.left, self.mode, self.right))

    def sample(self, n, d=None, rng=np.random):
        if d is None:
            return rng.triangular(self.left, self.mode, self.right, size=n)
        else:
            return rng.triangular(self.left, self.mode, self.right, size=(n, d))
