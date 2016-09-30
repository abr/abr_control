import numpy as np

from . import osc
# from .keeplearningsolver import KeepLearningSolver

# try:
#     import nengo
# except ImportError:
#     print('Nengo module needs to be installed to use this controller.')
#
# nengo_ocl = None
# try:
#     import nengo_ocl
# except ImportError:
#     print('Nengo OCL not installed, simulation will be slower.')


class controller(osc.controller):
    """ Extension of the OSC controller that incorporates dynamics
    adaptation using a Nengo model
    """

    def __init__(self, robot_config, pes_learning_rate=1e-6,
                 weights_file=None):
        """
        pes_learning_rate float: controls the speed of neural adaptation
                                 for training the dynamics compensation term
        weights_file string: path to file where learned weights are saved
        """

        super(controller, self).__init__(robot_config)

        self.u_adapt = np.zeros(self.robot_config.num_joints)

        # # Build the nengo model ---------------------
        # dim = self.robot_config.num_joints
        # nengo_model = nengo.Network()
        # with nengo_model:
        #
        #     def training_signal_func(t):
        #         """ returns the control signal for training """
        #         return self.training_signal
        #     node_training_signal = nengo.Node(training_signal_func,
        #                                       size_out=dim)
        #
        #     def kinadapt_output(t, x):
        #         robot_config.L_hat = np.copy(x)
        #     output = nengo.Node(output=kinadapt_output,
        #                         size_in=dim, size_out=0)
        #
        #     adapt_ens = nengo.Ensemble(**self.robot_config.adapt)
        #
        #     conn_learn = \
        #         nengo.Connection(
        #             adapt_ens, output,
        #             # start with outputting just L_init
        #             function=lambda x: robot_config.L_hat,
        #             learning_rule_type=nengo.PES(pes_learning_rate),)
        #             # use the weights solver that lets you keep
        #             # learning from the what's saved to file
        #             # solver=KeepLearningSolver(filename=weights_file))
        #     nengo.Connection(node_training_signal, conn_learn.learning_rule,
        #                      # invert because we're providing error not reward
        #                      transform=-1, synapse=.01)
        #
        #     # self.probe_weights = nengo.Probe(conn_learn, 'weights')
        #
        # if nengo_ocl is not None:
        #     self.sim = nengo_ocl.Simulator(nengo_model, dt=.001)
        # else:
        #     self.sim = nengo.Simulator(nengo_model, dt=.001)
        # # -----------------------------------------

        # low pass filtered Yk matrix
        self.Wk = np.zeros((3, len(self.robot_config.L_hat)))
        # estimate of end-effector velocity
        self.y = np.zeros(3)
        # low pass filtered hand position
        self.xyz_lp = np.zeros(3)

        # parameters from experiment 1 of cheah and slotine, 2005
        self.kp = 400
        self.kv = 100
        self.learning_rate = .04 * 1e-2
        self.alpha = 1.2
        self.lamb = 200.0 * np.pi

    def control(self, q, dq, target_xyz):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_xyz np.array: the current target for the end-effector
        """

        # calculate position of the end-effector
        self.xyz = self.robot_config.T('EE', q=q)

        # calculate desired force in (x,y,z) space
        delta_x = self.xyz - target_xyz

        # calculate dx using a low pass filter over x and taking
        # the difference from the current value of x
        self.xyz_lp += (self.lamb * (self.xyz - self.xyz_lp)) * .001
        self.y = self.lamb * (self.xyz - self.xyz_lp)

        # calculate Yk, the set of basis functions such that
        # np.dot(Yk, L) = np.dot(J, dq)
        Yk = self.robot_config.Y(q=q, dq=dq)
        # create a lowpass filter of Yk
        self.Wk += (self.lamb * (Yk - self.Wk)) * .001

        # calculate our update to the estimated arm segment lengths
        dL_hat = np.dot(
            self.learning_rate,
            (-np.dot(self.Wk.T,
                     np.dot(
                         self.kv,
                         (np.dot(self.Wk, self.robot_config.L_hat) - self.y))) +
             np.dot(Yk.T,
                    np.dot(self.kp + self.alpha * self.kv,
                           delta_x))))
        # put reasonable boundaries around L values
        dL_hat[(self.robot_config.L_hat + dL_hat) < .001] = 0.0
        dL_hat[(self.robot_config.L_hat + dL_hat) > 1] = 0.0

        # run the model, update the parameter estimations
        # self.sim.run(dt=.001)
        self.robot_config.L_hat += self.learning_rate * dL_hat

        # calculate the Jacobian for the end effector
        J = self.robot_config.J('EE', q=q)

        u = -np.dot(J.T, self.kp * delta_x + self.kv * self.y)
        return u
