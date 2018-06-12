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
from abr_control.utils import DataHandler
import abr_jaco2
import nengo

class Training:

    def __init__(self, adapt_input, adapt_output, n_neurons=1000, n_ensembles=1,
            test_group='testing', test_name="joint_space_training", session=None,
            run=None, weights=None, pes_learning_rate=1e-6, backend=None,
            offset=None, kp=20, kv=6, ki=0, seed=None, SCALES=None, MEANS=None,
            probe_weights=True, avoid_limits=True, neuron_type='lif'):
        """
        The test script used to collect data for training. The use of the
        adaptive controller and the type of backend can be specified. The
        script is also automated to create new and load the most recent weights
        files for continuous learning.

        We refere to 'runs' as the consecutive tests where the previous learned
        weights are used.  A set of runs are a single 'session'. Multiple
        sessions of these runs can then be used for averaging and analysis.

        The adaptive controller uses a 6 dimensional input and returns a 3
        dimensional output. The base, shoulder and elbow joints' positions and
        velocities are used as the input, and the control signal for the
        corresponding joints gets returned.

        Parameters
        ----------
        n_neurons: int, Optional (Default: 1000)
            the number of neurons in the adaptive population
        n_ensembles: int, Optional (Default: 1)
            the number of ensembles of n_neurons number of neurons
        test_group: string, Optional (Default: 'testing')
        test_name: string, Optional (Default: 'joint_space_training')
            folder name where data is saved
        session: int, Optional (Default: None)
            The current session number, if left to None then it will be automatically
            updated based on the current session. If the next session is desired then
            this will need to be updated.
        run: int, Optional (Default: None)
            The current nth run that specifies to use the weights from the n-1 run.
            If set to None if will automatically increment based off the last saved
            weights.
        weights: string, Optional (Default: None)
            the path to the desired saved weights to use. If None will
            automatically take the most recent weights saved in the 'test_name'
            directory
        pes_learning_rate: float, Optional (Default: 1e-6)
            the learning rate for the adaptive population
        backend: string
            None: non adaptive control, Optional (Default: None)
            'nengo': use nengo as the backend for the adaptive population
            'nengo_spinnaker': use spinnaker as the adaptive population
        time_limit: float, Optional (Default: 30)
            the time limit for each run in seconds
        offset: float array, Optional (Default: None)
            Set the offset to the end effector if something other than the default
            is desired. Use the form [x_offset, y_offset, z_offset]
        kp: float, Optional (Default: 20)
            proportional gain term
        kv: float, Optional (Default: 6)
            derivative gain term
        ki: float, Optional (Default: 0)
            integral gain term
        seed: int Optional, (Default: None)
            seed used for random number generation in adaptive population
        MEANS : list of floats, Optional (Default: None)
            expected mean of joint angles and velocities in [rad] and [rad/sec]
            respectively. Expected value for each joint. Only used for adaptation
        SCALES : list of floats, Optional (Default: None)
            expected variance of joint angles and velocities. Expected value for
            each joint. Only used for adaptation
        adapt_input : list of booleans
            a boolean list of which joint information to pass in to the
            adaptive population
            Ex: to use joint 1 and 2 information for a 6DOF arm
            (starting from base 0)
            adapt_input = [False, True, True, False, False, False]
        adapt_output : list of booleans
            a boolean list of which joints to apply the adaptive signal to
            Ex: to adapt joint 1 and 2 for a 6DOF arm (starting from base 0)
            adapt_input = [False, True, True, False, False, False]
        *NOTE: adapt_input DOES NOT have to be the same as adapt_input*
        probe_weights: Boolean Optional (Default: False)
            True to probe adaptive population for decoders
        avoid_limits: Boolean Optional (Default: True)
            Adds joint avoidance controller to prevent the arm from colliding
            with the floor and itself
            NOTE: this is only for joints1 and 2, it may still be possible for
            the gripper to collide with the arm in certain configurations

        Attributes
        ----------
        target_xyz: array of floats
            the goal target, used for calculating error
        target: array of floats [x, y, z, vx, vy, vz]
            the filtered target, used for calculating error for figure8
            test since following trajectory and not moving to a single final
            point
        """
        self.run = run
        self.session = session
        self.test_group = test_group
        self.test_name = test_name
        self.avoid_limits = avoid_limits
        print("RUN PASSED IN IS: ", self.run)

        # instantiate our data handler for saving and load
        self.data_handler = DataHandler(use_cache=True)

        # load previous weights if they exist is None passed in
        if weights is None:
            weights = self.data_handler.load_data(params=['weights'], session=self.session,
                    run=self.run, test_group=self.test_group,
                    test_name=self.test_name, create=True)

        print('--Instantiate robot_config--')
        # instantiate our robot config
        self.robot_config = abr_jaco2.Config(use_cython=True, hand_attached=True,
                SCALES=SCALES, MEANS=MEANS, init_all=True, offset=offset)

        if offset is None:
            self.OFFSET = self.robot_config.OFFSET
        else:
            self.OFFSET = offset
        print('--Generate / load transforms--')
        self.robot_config.init_all()

        print('--Instantiate adapt controller--')
        # instantiate our adaptive controller
        self.adapt = signals.DynamicsAdaptation(
            n_input=sum(adapt_input*2),
            n_output=sum(adapt_output),
            n_neurons=n_neurons,
            n_ensembles=n_ensembles,
            pes_learning_rate=pes_learning_rate,
            intercepts=(-0.9, 0),
            weights_file=weights,
            backend=backend,
            probe_weights=probe_weights,
            seed=seed,
            neuron_type=neuron_type)

        print('--Instantiate OSC controller--')
        # instantiate operational space controller
        self.ctrlr = OSC(robot_config=self.robot_config, kp=kp, kv=kv, ki=ki,
                vmax=1, null_control=True)

        print('--Instantiate path planner--')
        # instantiate our filter to smooth out trajectory to final target
        self.path = path_planners.SecondOrder(robot_config=self.robot_config,
                n_timesteps=4000, w=1e4, zeta=2, threshold=0.05)
        self.dt = 0.003

        if self.avoid_limits:
            print('--Instantiate joint avoidance--')
            self.avoid = signals.AvoidJointLimits(
                self.robot_config,
                min_joint_angles=[None, 1.3, 0.61, None, None, None],
                max_joint_angles=[None, 4.99, 5.75, None, None, None],
                max_torque=[5]*self.robot_config.N_JOINTS,
                cross_zero=[True, False, False, False, True, False],
                gradient = [False, False, False, False, False, False])

        print('--Instantiate interface--')
        # instantiate our interface
        self.interface = abr_jaco2.Interface(robot_config=self.robot_config)

        # set up lists for tracking data
        self.data = {'q': [], 'dq': [], 'u_base': [], 'u_adapt': [],
                     'error':[], 'training_signal': [], 'target': [],
                     'ee_xyz': [], 'input_signal': [], 'filter': [],
                     'time': [], 'weights': [], 'u_avoid': []}
        self.count = 0

        self.adapt_input = np.array(adapt_input)
        self.in_index = np.arange(self.robot_config.N_JOINTS)[self.adapt_input]
        self.adapt_output = np.array(adapt_output)
        self.out_index = np.arange(self.robot_config.N_JOINTS)[self.adapt_output]

    def connect_to_arm(self):
        # connect to and initialize the arm
        print('--Connect to arm--')
        self.interface.connect()
        self.interface.init_position_mode()
        self.interface.send_target_angles(self.robot_config.INIT_TORQUE_POSITION)
        print('--Initialize force mode--')
        self.interface.init_force_mode()

    def reach_to_target(self, target_xyz, reaching_time):
        # TODO: ADD NOTE ABOUT USING BASH FOR MULTIPLE RUNS INSTEAD OF JUST
        # CALLING RUN, MENTION PERFORMANCE EFFECTS ETC
        """

        Parameters
        ----------
        target_xyz: numpy array of floats
            x,y,z position of target [meters]
        reaching_time: float
            time [seconds] to reach to target
        """

        # track loop_time for stopping test
        loop_time = 0

        # get joint angle and velocity feedback to reset starting point to
        # current end-effector position
        feedback = self.interface.get_feedback()
        self.q = feedback['q']
        self.dq = feedback['dq']
        self.dq[abs(self.dq) < 0.05] = 0
        # calculate end-effector position
        ee_xyz = self.robot_config.Tx('EE', q=self.q, x= self.OFFSET)

        # last three terms used as started point for target EE velocity
        self.target = np.concatenate((ee_xyz, np.array([0, 0, 0])), axis=0)

        print('--Starting main control loop--')
        # M A I N   C O N T R O L   L O O P
        while loop_time < reaching_time:
            start = timeit.default_timer()
            prev_xyz = ee_xyz

            # use our filter to get the next point along the trajectory to our
            # final target location
            self.target = self.path.step(state=self.target, target_pos=target_xyz,
                    dt=self.dt)

            # calculate euclidean distance to target
            error = np.sqrt(np.sum((ee_xyz - target_xyz)**2))

            # get joint angle and velocity feedback
            feedback = self.interface.get_feedback()
            self.q = feedback['q']
            self.dq = feedback['dq']
            self.dq[abs(self.dq) < 0.05] = 0

            # calculate end-effector position
            ee_xyz = self.robot_config.Tx('EE', q=self.q, x= self.OFFSET)

            # Calculate the control signal and the adaptive signal
            u = self.generate_u()

            # send forces
            self.interface.send_forces(np.array(u, dtype='float32'))

            # track data
            self.data['q'].append(np.copy(self.q))
            self.data['dq'].append(np.copy(self.dq))
            self.data['u_base'].append(np.copy(self.u_base))
            self.data['u_adapt'].append(np.copy(self.u_adapt))
            self.data['u_avoid'].append(np.copy(self.u_avoid))
            self.data['error'].append(np.copy(error))
            self.data['training_signal'].append(np.copy(self.training_signal))
            end = timeit.default_timer() - start
            self.data['time'].append(np.copy(end))
            self.data['target'].append(np.copy(target_xyz))
            self.data['input_signal'].append(np.copy(self.adapt_input))
            self.data['ee_xyz'].append(np.copy(ee_xyz))
            self.data['filter'].append(np.copy(self.target))

            if self.count % 1000 == 0:
                print('error: ', error)
                print('dt: ', end)
                print('adapt: ', self.u_adapt)

            loop_time += end
            self.count += 1

    def generate_u(self):
            # calculate the base operation space control signal
            self.u_base = self.ctrlr.generate(
                q=self.q,
                dq=self.dq ,
                target_pos=self.target[:3],
                target_vel=self.target[3:],
                offset = self.OFFSET)

            # account for uneven stiction in jaco2 base
            if self.u_base[0] > 0:
                self.u_base[0] *= 3.0
            else:
                self.u_base[0] *= 2.0

            # calculate the adaptive control signal
            training_signal = []
            adapt_input_q = []
            adapt_input_dq = []

            for ii in self.in_index:
                training_signal.append(self.ctrlr.training_signal[ii])
                adapt_input_q.append(self.robot_config.scaledown('q',self.q)[ii])
                adapt_input_dq.append(self.robot_config.scaledown('dq',self.dq)[ii])

            self.training_signal = np.array(training_signal)
            self.adapt_input = np.hstack((adapt_input_q, adapt_input_dq))

            u_adapt = self.adapt.generate(input_signal=self.adapt_input,
                                     training_signal=self.training_signal)

            # create array of zeros, we will change the adaptive joints with
            # their respective outputs. This just puts the adaptive output into
            # the same shape as our base control
            self.u_adapt = np.zeros(self.robot_config.N_JOINTS)
            count = 0
            for ii in self.out_index:
                self.u_adapt[ii] = u_adapt[count]
                count += 1

            # add adaptive signal to base controller
            u = self.u_base + self.u_adapt

            if self.avoid_limits:
                # add in joint limit avoidance
                self.u_avoid = self.avoid.generate(self.q)
                u += self.u_avoid
            else:
                self.u_avoid = None

            return u

    def stop(self):
        print('--Disconnecting--')
        # close the connection to the arm
        self.interface.init_position_mode()
        self.interface.send_target_angles(self.robot_config.INIT_TORQUE_POSITION)
        self.interface.disconnect()

        print('**** RUN STATS ****')
        print('Number of steps: ', self.count)
        print('Average loop speed: ',
              sum(self.data['time'])/len(self.data['time']))
        print('Average Error/Step: ',
              sum(self.data['error'])/len(self.data['error']))
        print('Run number ', self.run)
        print('*******************')

    def save_data(self):
        print('--Saving run data--')
        # Get the last saved location in the database
        [self.run, self.session, location] = self.data_handler.last_save_location(
            session=self.session, run=self.run, test_name=self.test_name,
            test_group=self.test_group, create=False)
        if self.run is None:
            self.run = 0
        else:
            self.run += 1
        print('Saving tracked data to %s/%s/session%i/run%i'%(self.test_group, self.test_name,
            self.session, self.run))

        # Get weight from adaptive population
        self.data['weights'] = self.adapt.get_weights()

        # Save test data
        self.data_handler.save_data(tracked_data=self.data, session=self.session,
            run=self.run, test_name=self.test_name, test_group=self.test_group)

    def save_parameters(self, overwrite=True, create=True):
        print('--Saving test parameters--')
        loc = '%s/%s/'%(self.test_group, self.test_name)
        # Save OSC parameters
        self.data_handler.save(data=self.ctrlr.params,
                save_location=loc + self.ctrlr.params['source'], overwrite=overwrite, create=create)

        # Save dynamics_adaptation parameters
        self.data_handler.save(data=self.adapt.params,
                save_location=loc + self.adapt.params['source'], overwrite=overwrite, create=create)

        # Save robot_config parameters
        self.data_handler.save(data=self.robot_config.params,
                save_location=loc + self.robot_config.params['source'], overwrite=overwrite, create=create)

        # Save path planner parameters
        self.data_handler.save(data=self.path.params,
                save_location=loc + self.path.params['source'], overwrite=overwrite, create=create)
