import numpy as np
import scipy

try:
    import nengo
except ImportError:
    raise Exception('Nengo module needs to be installed to ' +
                    'use adaptive dynamics.')

nengo_ocl = None
try:
    import nengo_ocl
except ImportError:
    print('Nengo OCL not installed, simulation will be slower.')

nengolib = None
try:
    import nengolib
except ImportError:
    print('Nengo lib not installed, encoder placement will be sub-optimal.')

from abr_control.utils.keeplearningsolver import KeepLearningSolver

class AreaIntercepts(nengo.dists.Distribution):
    """ A Nengo distribution for distributing intercepts more effectively
    in high-dimensional spaces, such that they are spread out to be more
    evenly active throughout the state space. """
    dimensions = nengo.params.NumberParam('dimensions')
    base = nengo.dists.DistributionParam('base')

    def __init__(self, dimensions, base=nengo.dists.Uniform(-1, 1)):
        super(AreaIntercepts, self).__init__()
        self.dimensions = dimensions
        self.base = base

    def __repr(self):
        return "AreaIntercepts(dimensions=%r, base=%r)" % (self.dimensions, self.base)

    def transform(self, x):
        sign = 1
        if x > 0:
            x = -x
            sign = -1
        return sign * np.sqrt(1-scipy.special.betaincinv((self.dimensions+1)/2.0, 0.5, x+1))

    def sample(self, n, d=None, rng=np.random):
        s = self.base.sample(n=n, d=d, rng=rng)
        for i in range(len(s)):
            s[i] = self.transform(s[i])
        return s


class Signal():
    """ An implementation of dynamics adaptation using a Nengo model
    """

    def __init__(self, robot_config, n_neurons=1000,
                 pes_learning_rate=1e-6,
                 voja_learning_rate=1e-6,
                 weights_file=None,
                 encoders_file=None):
        """
        pes_learning_rate float: controls the speed of neural adaptation
                                 for training the dynamics compensation term
        voja_learning_rate float: controls the speed of neural adaptation
                                  for shifting the sensitivity of the CB
                                  encoders towards areas most often explored
        weights_file string: path to file where learned weights are saved
        encoders_file string: path to file where learned encoders are saved
        """

        self.robot_config = robot_config

        self.u_adapt = np.zeros(self.robot_config.num_joints)

        dim = self.robot_config.num_joints
        nengo_model = nengo.Network()
        with nengo_model:

            def qdq_input(t):
                """ returns q and dq scaled and bias to
                be around -1 to 1 """
                q = ((self.q + np.pi) % (np.pi*2)) - np.pi

                output = np.hstack([
                    self.robot_config.scaledown('q', q),
                    self.robot_config.scaledown('dq', self.dq)])
                return output
            qdq_input = nengo.Node(qdq_input, size_out=dim*2)

            def u_input(t):
                """ returns the control signal for training """
                return self.training_signal
            u_input = nengo.Node(u_input, size_out=dim)

            def u_adapt_output(t, x):
                """ stores the adaptive output for use in control() """
                self.u_adapt = np.copy(x)
            output = nengo.Node(u_adapt_output, size_in=dim, size_out=0)

            eval_points = (nengolib.stats.ScatteredHypersphere(surface=False) if
                nengolib is not None else None)
            encoders = (nengolib.stats.ScatteredHypersphere(surface=True) if
                nengolib is not None else None)
            adapt_ens = nengo.Ensemble(
                seed=10,
                n_neurons=n_neurons,
                dimensions=self.robot_config.num_joints * 2,
                encoders=encoders,
                eval_points=eval_points,
                intercepts=AreaIntercepts(
                    self.robot_config.num_joints * 2,
                    nengo.dists.Uniform(-.2, 1)),
                radius=np.sqrt(self.robot_config.num_joints * 2))

            if encoders_file is not None:
                try:
                    encoders = np.load(encoders_file)['encoders'][-1]
                    adapt_ens.encoders = encoders
                    print('\nLoaded encoders from %s\n' % encoders_file)
                except Exception:
                    print('\nNo encoders file found, generating normally\n')
                    pass

            # connect input to CB with Voja so that encoders shift to
            # most commonly explored areas of state space
            conn_in = nengo.Connection(
                qdq_input,
                adapt_ens[:self.robot_config.num_joints * 2],)
                # learning_rule_type=nengo.Voja(voja_learning_rate))

            conn_learn = \
                nengo.Connection(
                    adapt_ens[:self.robot_config.num_joints * 2], output,
                    # start with outputting just zero
                    function=lambda x: np.zeros(dim),
                    learning_rule_type=nengo.PES(pes_learning_rate),
                    # use the weights solver that lets you keep
                    # learning from the what's saved to file
                    solver=KeepLearningSolver(filename=weights_file))
            nengo.Connection(u_input, conn_learn.learning_rule,
                             # invert because we're providing error not reward
                             transform=-1, synapse=.01)

            self.probe_weights = nengo.Probe(conn_learn, 'weights')
            # self.probe_encoders = nengo.Probe(conn_in.learning_rule, 'scaled_encoders')
            # self.probe_neurons = nengo.Probe(adapt_ens.neurons, sample_every=.002)

        if nengo_ocl is not None:
            self.sim = nengo_ocl.Simulator(nengo_model, dt=.001)
        else:
            self.sim = nengo.Simulator(nengo_model, dt=.001)

    def generate(self, q, dq, training_signal):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        training_signal np.array: the learning signal to drive adaptation
        """

        # store local copies to feed in to the adaptive population
        self.q = q
        self.dq = dq
        self.training_signal = training_signal

        # run the simulation to generate the adaptive signal
        self.sim.run(time_in_seconds=.001, progress_bar=False)

        return self.u_adapt
