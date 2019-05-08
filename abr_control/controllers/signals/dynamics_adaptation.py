import numpy as np

import nengo
from nengolib.stats import spherical_transform

import scipy.special

class DynamicsAdaptation():
    """ An implementation of nonlinear dynamics adaptation using Nengo,
    as described in (DeWolf, Stewart, Slotine, and Eliasmith, 2016)

    The model learns to account for unknown / unmodelled external or
    internal forces, using an efferent copy of the control signal to train.

    Parameters
    ----------
    n_input: int
        the number of inputs going into the adaptive population
    n_output: int
        the number of outputs expected from the adaptive population
    n_neurons: int, optional (Default: 1000)
        number of neurons per adaptive population
    n_ensembles: int, optional (Default: 1)
        number of ensembles of n_neurons number of neurons
    seed: int, optional (Default: None)
        the seed used for random number generation
    pes_learning_rate: float, optional (Default: 1e-6)
        controls the speed of neural adaptation
    intercepts: np.array, optional (Default: None)
        the neural intercepts to be used by the neural ensembles
    intercepts_bounds: list, optional (Default: [-0.9, 0.0])
        the upper and lower bounds of the AreaIntercepts distribution
        for selecting intercepts
    intercepts_mode: scalar, optional (Default: -0.5)
        the desired mode of the AreaIntercepts distribution used for selecting
        intercepts
    encoders: np.array, optional (Default: None)
        an (n_encoders, n_neurons, n_input) array of preferred directions
        for all of the neurons in the adaptive ensembles
    spherical: boolean, Optional (Default: False)
        True to convert inputs to the surface of the hypersphere
        False to scale from 0 to 1
    MEANS: dict of floats
        MEANS['q'] == the mean expected values of the joint positions [rad]
        MEANS['dq'] == the mean expected values of the joint velocities [rad/s]
    VARIANCES: dict of floats
        VARIANCES['q'] == the abs(max) variance from the MEANS['q'] [rad]
        VARIANCES['dq'] == the abs(max) variance from the MEANS['dq'] [rad/s]

        ***NOTE*** The VARIANCES do not have to encompass the actual abs(max)
        joint positions or velocities. Outliers can be left outside the scaling
        range to get a better scaling of the majority of your expected inputs.
        The outliers will still be scaled, but they will be outside the 0-1
        range for non-spherical, or 0-1 range for spherical
    """

    def __init__(self, n_input, n_output, n_neurons=1000, n_ensembles=1,
                 seed=None, pes_learning_rate=1e-6, intercepts=None,
                 intercepts_bounds=None, intercepts_mode=None,
                 weights=None, encoders=None, spherical=False, MEANS=None,
                 VARIANCES=None, **kwargs):

        self.n_neurons = n_neurons
        self.n_ensembles = n_ensembles
        self.spherical = spherical
        self.MEANS = MEANS
        self.VARIANCES = VARIANCES
        # synapse time constants
        self.tau_input = 0.012  # on input connection
        self.tau_training = 0.012  # on the training signal
        self.tau_output = 0.2  # on the output from the adaptive ensemble
        #NOTE: the time constant on the neural activity used in the learning
        # connection is the default 0.005, and can be set by specifying the
        # pre_synapse parameter inside the PES rule instantiation

        self.seed = seed
        self.pes_learning_rate = pes_learning_rate

        if intercepts is None:
            # set up neuron intercepts
            if intercepts_bounds is None:
                intercepts_bounds = [-0.9, 0.0]
            self.intercepts_bounds = intercepts_bounds

            if intercepts_mode is None:
                intercepts_mode = -0.5
            self.intercepts_mode = intercepts_mode

            intercepts = AreaIntercepts(dimensions=n_input, base=Triangular(
                intercepts_bounds[0], intercepts_mode, intercepts_bounds[1]))

        self.intercepts = intercepts

        if weights is None:
            weights = np.zeros((self.n_ensembles, n_output, self.n_neurons))
            print('Initializing connection weights to all zeros')

        if encoders is None:
            # if NengoLib is installed, use it to optimize encoder placement
            try:
                import nengolib
                encoders = [
                    nengolib.stats.ScatteredHypersphere(surface=True)
                    for ii in range(self.n_ensembles)]
            except ImportError:
                encoders = [nengo.Default for ii in range(self.n_ensembles)]
                print('NengoLib not installed, encoder placement will ' +
                      'be sub-optimal.')
        self.encoders = encoders

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
            for ii in range(self.n_ensembles):
                self.adapt_ens.append(
                    nengo.Ensemble(
                        n_neurons=self.n_neurons,
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
                        transform=weights[ii],
                        synapse=self.tau_output,
                        )
                    )

                # hook up the training signal to the learning rule
                nengo.Connection(
                    training_signals, self.conn_learn[ii].learning_rule,
                    synapse=self.tau_training)

        nengo.rc.set("decoder_cache", "enabled", "False")
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

        if self.MEANS is not None and self.VARIANCES is not None:
            input_signal = self.scale_inputs(input_signal)

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


    def scale_inputs(self, input_signal):
        '''
        Currently set to accept joint position and velocities as time
        x dimension arrays, and returns them scaled based on the MEANS and
        VARIANCES set on instantiation

        PARAMETERS
        ----------
        input_signal : numpy.array
            the current desired input signal, typical joint positions and
            velocities in [rad] and [rad/sec] respectively

        The reason we do a shift by the MEANS is to try and center the majority
        of our expected input values in our scaling range, so when we stretch
        them by VARIANCES they encompass more of our input range.
        '''
        scaled_input = (input_signal - self.MEANS) / self.VARIANCES

        if self.spherical:
            scaled_input = spherical_transform(
                scaled_input.reshape(1, len(scaled_input)))

        return scaled_input


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
