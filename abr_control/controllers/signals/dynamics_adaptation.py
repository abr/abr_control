import numpy as np

import nengo

from abr_control._vendor.nengolib.stats import ScatteredHypersphere, spherical_transform


class DynamicsAdaptation:
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
        shape (n_ensembles, n_neurons)
    encoders: np.array, optional (Default: None)
        an (n_encoders, n_neurons, n_input) array of preferred directions
        for all of the neurons in the adaptive ensembles
    spherical: boolean, Optional (Default: False)
        True to convert inputs to the surface of the hypersphere
        False to scale from 0 to 1
    means: np.array, optional (Default: None)
        Subtracted from the input to the neural ensemble to center the data
        for each dimension around zero
    variances: np.array, optional (Default: None)
        The input signal to the neural ensemble is divided by these values
        after mean subtraction to put the data for each dimensions in the
        range -1 to 1, or 0 to 1 if spherical=True

        ***NOTE*** The variances do not have to encompass the full range of
        joint positions or velocities. Outliers can be left outside the scaling
        range to get a better scaling of the majority of your expected inputs.
        The outliers will still be scaled, but they will be outside the -1 to 1
        range for non-spherical, or 0 to 1 range for spherical
    """

    def __init__(
        self,
        n_input,
        n_output,
        n_neurons=1000,
        n_ensembles=1,
        seed=None,
        pes_learning_rate=1e-6,
        intercepts=None,
        weights=None,
        encoders=None,
        spherical=False,
        means=None,
        variances=None,
        tau_input=0.012,
        tau_training=0.012,
        tau_output=0.2,
        **kwargs
    ):

        self.n_neurons = n_neurons
        self.n_ensembles = n_ensembles
        if spherical:
            n_input += 1
        self.spherical = spherical

        # if only one of means or variances is defined
        # define the other to have no effect on the data
        if means is not None and variances is None:
            variances = np.ones(means.shape)
        elif means is None and variances is not None:
            means = np.zeros(variances.shape)
        self.means = np.asarray(means)
        self.variances = np.asarray(variances)

        # synapse time constants
        self.tau_input = 0.012  # on input connection
        self.tau_training = 0.012  # on the training signal
        self.tau_output = 0.2  # on the output from the adaptive ensemble
        # NOTE: the time constant on the neural activity used in the learning
        # connection is the default 0.005, and can be set by specifying the
        # pre_synapse parameter inside the PES rule instantiation

        self.seed = seed
        self.pes_learning_rate = pes_learning_rate

        if intercepts is None:
            # Generates intercepts for a d-dimensional ensemble, such that, given a
            # random uniform input (from the interior of the d-dimensional ball), the
            # probability of a neuron firing has the probability density function given
            # by rng.triangular(left, mode, right, size=n)
            triangular = np.random.triangular(
                left=0.35, mode=0.45, right=0.55, size=n_neurons * n_ensembles
            )
            intercepts = nengo.dists.CosineSimilarity(n_input + 2).ppf(1 - triangular)
            intercepts = intercepts.reshape((n_ensembles, n_neurons))

        if weights is None:
            weights = np.zeros((self.n_ensembles, n_output, self.n_neurons))
            print("Initializing connection weights to all zeros")

        if encoders is None:
            np.random.seed = self.seed
            # if NengoLib is installed, use it to optimize encoder placement
            try:
                encoders_dist = ScatteredHypersphere(surface=True)
            except ImportError:
                encoders_dist = nengo.Default
                print(
                    "NengoLib not installed, encoder placement will "
                    + "be sub-optimal."
                )
            encoders = encoders_dist.sample(n_neurons * n_ensembles, n_input)
            encoders = encoders.reshape(n_ensembles, n_neurons, n_input)

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

            training_signals = nengo.Node(training_signals_func, size_out=n_output)

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
                        intercepts=intercepts[ii],
                        radius=np.sqrt(n_input),
                        encoders=encoders[ii],
                        **kwargs,
                    )
                )

                # hook up input signal to adaptive population to provide context
                nengo.Connection(
                    input_signals, self.adapt_ens[ii], synapse=self.tau_input,
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
                    training_signals,
                    self.conn_learn[ii].learning_rule,
                    synapse=self.tau_training,
                )

        nengo.rc.set("decoder_cache", "enabled", "False")
        self.sim = nengo.Simulator(self.nengo_model, dt=0.001)

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

        # if means or variances was defined, self.means is not None
        if self.means is not None:
            input_signal = self.scale_inputs(input_signal)

        # store local copies to feed in to the adaptive population
        self.input_signal = input_signal
        self.training_signal = training_signal

        # run the simulation t generate the adaptive signal
        self.sim.run(time_in_seconds=0.001, progress_bar=False)

        return self.output

    def scale_inputs(self, input_signal):
        """
        Currently set to accept joint position and velocities as time
        x dimension arrays, and returns them scaled based on the means and
        variances set on instantiation

        PARAMETERS
        ----------
        input_signal : numpy.array
            the current desired input signal, typical joint positions and
            velocities in [rad] and [rad/sec] respectively

        The reason we do a shift by the means is to try and center the majority
        of our expected input values in our scaling range, so when we stretch
        them by variances they encompass more of our input range.
        """
        scaled_input = (input_signal - self.means) / self.variances

        if self.spherical:
            # put into the 0-1 range
            scaled_input = scaled_input / 2 + 0.5
            # project onto unit hypersphere in larger state space
            scaled_input = scaled_input.flatten()
            scaled_input = spherical_transform(
                scaled_input.reshape(1, len(scaled_input))
            )

        return scaled_input

    def get_weights(self):
        """ Save the current weights to the specified test_name folder

        Save weights for individual runs. A group of runs is
        classified as a session. Multiple sessions can then be used
        to average over a set of learned runs. If session or run are set to None
        then the test_name location will be searched for the highest numbered
        folder and file respectively
        """

        return [
            self.sim.signals[self.sim.model.sig[conn]["weights"]]
            for conn in self.conn_learn
        ]
