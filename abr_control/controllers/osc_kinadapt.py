import numpy as np

from . import osc
from .keeplearningsolver import KeepLearningSolver

try:
    import nengo
except ImportError:
    print('Nengo module needs to be installed to use this controller.')

# nengo_ocl = None
# try:
#     import nengo_ocl
# except ImportError:
#     print('Nengo OCL not installed, simulation will be slower.')


class controller(osc.controller):
    """ Extension of the OSC controller that incorporates dynamics
    adaptation using a Nengo model
    """

    def __init__(self, robot_config,
                 voja_learning_rate=1e-6,
                 weights_file=None, encoders_file=None):
        """
        pes_learning_rate float: controls the speed of neural adaptation
                                 for training the dynamics compensation term
        weights_file string: path to file where learned weights are saved
        """

        super(controller, self).__init__(robot_config)

        self.voja_learning_rate = voja_learning_rate
        self.weights_file = weights_file
        self.encoders_file = encoders_file

        self.u_adapt = np.zeros(self.robot_config.num_joints)

        self.build_dynadapt()

        # low pass filtered Yk matrix
        self.Wk = np.zeros((3, len(self.robot_config.L_hat)))
        # low pass filtered hand position
        self.xyz_lp = np.zeros(3)

        # parameters from experiment 1 of cheah and slotine, 2005
        self.kp = 2000
        self.kv = 500
        self.learn_rate_k = 0.0# np.diag([0.04, 0.045]) * 1e-2
        self.learn_rate_d = .0005 * 1
        self.alpha = 1.2
        self.lamb = 200.0 * np.pi

    def control(self, q, dq, target_xyz):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_xyz np.array: the current target for the end-effector
        """

        self.q = q
        self.dq = dq

        # calculate the _actual_ position of the end-effector
        # (assuming we have visual feedback or somesuch here)
        xyz = self.robot_config.T('EE', q=q, use_estimate=False)

        # calculate desired force in (x,y,z) space
        delta_x = xyz - target_xyz

        # calculate dx using a low pass filter over x and taking
        # the difference from the current value of x
        self.xyz_lp += (self.lamb * (xyz - self.xyz_lp)) * .001
        y = self.lamb * (xyz - self.xyz_lp)

        # calculate Yk, the set of basis functions such that
        # np.dot(Yk, L) = np.dot(J, dq)
        Yk = self.robot_config.Y(q=q, dq=dq)
        # create a lowpass filter of Yk
        self.Wk += (self.lamb * (Yk - self.Wk)) * .001

        # calculate our update to the estimated arm segment lengths
        dL_hat = np.dot(
            self.learn_rate_k,
            (np.dot(
                -self.Wk.T,
                np.dot(
                    self.kv,
                    np.dot(
                        self.Wk,
                        self.robot_config.L_hat) -
                     y)) +
             np.dot(
                 Yk.T,
                 np.dot(
                     self.kp + self.alpha * self.kv,
                     delta_x))))
        # put reasonable boundaries around L values
        dL_hat[(self.robot_config.L_hat + dL_hat) < .001] = 0.0
        dL_hat[(self.robot_config.L_hat + dL_hat) > 1] = 0.0

        # run the model, update the parameter estimations
        # self.sim.run(dt=.001)
        self.robot_config.L_hat += dL_hat

        # calculate the Jacobian for the end effector
        J_hat = self.robot_config.J('EE', q=q)

        self.training_signal = (
            -np.dot(
                self.learn_rate_d * 1000,
                np.dot(
                    np.linalg.pinv(J_hat),
                    np.dot(
                        Yk,
                        self.robot_config.L_hat) +
                    self.alpha * (xyz - target_xyz))))
        self.sim.run(.001, progress_bar=False)
        print('u_adapt: ', [float('%.3f' % val) for val in self.u_adapt])

        u = (-np.dot(J_hat.T,
                     self.kp * delta_x +
                     self.kv * np.dot(Yk,
                                      self.robot_config.L_hat))) + self.u_adapt
        return u

    def build_dynadapt(self):

        # Build the nengo model dynamics adaptation ------------------------
        dim = self.robot_config.num_joints
        nengo_model = nengo.Network()
        with nengo_model:

            def qdq_input(t):
                """ returns q and dq scaled and bias to
                be around -1 to 1 """
                q = ((self.q + np.pi) % (np.pi*2)) - np.pi
                return np.hstack([
                    self.robot_config.scaledown('q', q),
                    self.robot_config.scaledown('dq', self.dq)])
            qdq_input = nengo.Node(qdq_input, size_out=dim*2)

            def training_func(t):
                """ returns the control signal for training """
                return self.training_signal
            training_node = nengo.Node(training_func, size_out=dim)

            def u_adapt_output(t, x):
                """ stores the adaptive output for use in control() """
                self.u_adapt = np.copy(x)
            output = nengo.Node(u_adapt_output, size_in=dim, size_out=0)

            adapt_ens = nengo.Ensemble(seed=10, **self.robot_config.CB_adapt)
            if self.encoders_file is not None:
                try:
                    encoders = np.load(self.encoders_file)['encoders'][-1]
                    adapt_ens.encoders = encoders
                    print('Loaded encoders from %s' % self.encoders_file)
                except Exception:
                    print('No encoders file found, generating normally')
                    pass

            # connect input to CB with Voja so that encoders shift to
            # most commonly explored areas of state space
            conn_in = nengo.Connection(
                qdq_input,
                adapt_ens,)
                # learning_rule_type=nengo.Voja(self.voja_learning_rate))

            conn_learn = \
                nengo.Connection(
                    adapt_ens, output,
                    # start with outputting just zero
                    function=lambda x: np.zeros(dim),
                    learning_rule_type=nengo.PES(learning_rate=1.0),
                    # use the weights solver that lets you keep
                    # learning from the what's saved to file
                    solver=KeepLearningSolver(filename=self.weights_file))
            nengo.Connection(training_node, conn_learn.learning_rule,
                             # invert because we're providing error not reward
                             transform=-1, synapse=.01)

            self.probe_weights = nengo.Probe(conn_learn, 'weights')
            # self.probe_encoders = nengo.Probe(conn_in.learning_rule,
            #                                   'scaled_encoders')

        print('building adaptive dynamics Nengo model')
        self.sim = nengo.Simulator(nengo_model)
