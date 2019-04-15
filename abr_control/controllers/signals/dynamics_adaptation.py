import os
import numpy as np
import scipy.special

from abr_control.utils.paths import cache_dir

try:
    import nengo
    nengo.rc.set("decoder_cache", "enabled", "False")
except ImportError:
    raise Exception('Nengo module needs to be installed to ' +
                    'use adaptive dynamics.')


class DynamicsAdaptation():
    """ An implementation of nonlinear dynamics adaptation using Nengo,
    as described in (DeWolf, Stewart, Slotine, and Eliasmith, 2016)

    The model learns to account for unknown / unmodelled external or
    internal forces, using an efferent copy of the control signal to train.

    Parameters
    ----------
    n_input : int
        the number of inputs going into the adaptive population
    n_output : int
        the number of outputs expected from the adaptive population
    n_neurons : int, optional (Default: 1000)
        number of neurons per adaptive population
    n_ensembles : int, optional (Default: 1)
        number of ensembles of n_neurons number of neurons
    seed : int optional (Default: None)
        the seed used for random number generation
    pes_learning_rate : float, optional (Default: 1e-6)
        controls the speed of neural adaptation
    intercepts_bounds:

    intercepts_mode:

    weights_file : string, optional (Default: None)
        path to file where learned weights are saved
    encoders:

    probe_weights: boolean, optional (Default: False)
        True to get decoders
    neuron_type: string: optional (Default: 'lif')
        the neuron model to use
    intercepts:

    """

    def __init__(self, n_input, n_output, n_neurons=1000, n_ensembles=1,
                 seed=None, pes_learning_rate=1e-6,
                 intercepts_bounds=(-0.9, 0.0), intercepts_mode=-0.9,
                 weights_file='~', encoders=None, probe_weights=False,
                 neuron_type='lif', intercepts=None, **kwargs):

        self.tau_input = 0.012
        self.tau_training = 0.012
        self.tau_output = 0.2

        self.encoders = encoders
        self.seed = seed
        self.pes_learning_rate = pes_learning_rate

        # set up neuron intercepts
        self.intercepts_bounds = intercepts_bounds
        self.intercepts_mode = intercepts_mode
        if intercepts is None:
            intercepts = AreaIntercepts(dimensions=n_input, base=Triangular(
                intercepts_bounds[0], intercepts_mode, intercepts_bounds[1]))
        self.intercepts = intercepts

        # load weights from file if they exist, otherwise use zeros
        weights_file = os.path.expanduser(weights_file)
        if os.path.isfile(weights_file):
            transform = np.load(weights_file)['weights']
            print('Weights successfully loaded from %s' % weights_file)
        else:
            transform = np.zeros((n_ensembles, n_output, n_neurons))
            print('No weights found, starting with zeros')

        if self.encoders is None:
            # if NengoLib is installed, use it to optimize encoder placement
            try:
                import nengolib
                self.encoders = [
                    nengolib.stats.ScatteredHypersphere(surface=True)
                    for ii in range(n_ensembles)]
            except ImportError:
                self.encoders = [None for ii in range(n_ensembles)]
                print('NengoLib not installed, encoder placement will ' +
                      'be sub-optimal.')

        self.input_signal = np.zeros(n_input)
        self.training_signal = np.zeros(n_output)
        self.output = np.zeros(n_output)

        self.nengo_model = nengo.Network(seed=seed)
        # Set the default neuron type for the network
        self.nengo_model.config[nengo.Ensemble].neuron_type = nengo.LIF()
        with self.nengo_model:

            def input_signals_func(t):
                return self.input_signal
            input_signals = nengo.Node(input_signals_func, size_out=n_input)

            def training_signals_func(t):
                return -self.training_signal
            training_signals = nengo.Node(
                training_signals_func, size_out=n_output)

            # make the adaptive population output accessible
            def output_func(t, x):
                self.output = np.copy(x)
            output = nengo.Node(output_func, size_in=n_output, size_out=0)

            self.adapt_ens = []
            self.conn_learn = []
            for ii in range(n_ensembles):
                self.adapt_ens.append(
                    nengo.Ensemble(
                        n_neurons=n_neurons,
                        dimensions=n_input,
                        intercepts=intercepts,
                        radius=np.sqrt(n_input),
                        encoders=self.encoders[ii],
                        **kwargs))

                # hook up input signal to adaptive population to provide context
                nengo.Connection(
                    input_signals,
                    self.adapt_ens[ii],
                    synapse=self.tau_input,
                    )

                self.conn_learn.append(
                    nengo.Connection(
                        self.adapt_ens[ii].neurons,
                        output,
                        learning_rule_type=nengo.PES(pes_learning_rate),
                        transform=transform[ii],
                        synapse=self.tau_output,
                        )
                    )

                # hook up the training signal to the learning rule
                nengo.Connection(
                    training_signals, self.conn_learn[ii].learning_rule,
                    synapse=self.tau_training)

            if probe_weights:
                self.nengo_model.weights_probe = nengo.Probe(
                    self.conn_learn[0], 'weights', synapse=None)

        nengo.cache.DecoderCache().invalidate()
        self.sim = nengo.Simulator(self.nengo_model, dt=.001)


    def generate(self, input_signal, training_signal):
        """ Generates the control signal

        Parameters
        ----------
        input_signal : numpy.array
            the current desired input signal, typical joint positions and
            velocities in [rad] and [rad/sec] respectively
        training_signal : numpy.array
            the learning signal to drive adaptation
        """

        # store local copies to feed in to the adaptive population
        self.input_signal = input_signal
        self.training_signal = training_signal

        # run the simulation t generate the adaptive signal
        self.sim.run(time_in_seconds=.001, progress_bar=False)

        return self.output


    def get_weights(self):
        """ Save the current weights to the specified test_name folder

        Save weights for individual runs. A group of runs is
        classified as a session. Multiple sessions can then be used
        to average over a set of learned runs. If session or run are set to None
        then the test_name location will be searched for the highest numbered
        folder and file respectively
        """

        return [self.sim.signals[self.sim.model.sig[conn]['weights']]
                for conn in self.conn_learn]


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
        for ii, ss in enumerate(s):
            s[ii] = self.transform(ss)
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
            return rng.triangular(
                self.left, self.mode, self.right, size=(n, d))
