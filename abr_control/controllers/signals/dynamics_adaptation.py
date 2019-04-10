import glob
import redis
import struct
import os
import time
import numpy as np
import scipy.special
import logging
r = redis.StrictRedis(host='127.0.0.1')

# import abr_control.utils.os_utils
from abr_control.utils.paths import control_cache
from .signal import Signal
os.environ['PYOPENCL_COMPILER_OUTPUT'] = '1'
try:
    import nengo
    nengo.rc.set("decoder_cache", "enabled", "False")
except ImportError:
    raise Exception('Nengo module needs to be installed to ' +
                    'use adaptive dynamics.')
# try:
#     from nengo_extras.dists import Concatenate
# except ImportError:
#     raise Exception('Nengo_extras module needs to be installed to ' +
#                     'bootstrap learning.')


class DynamicsAdaptation(Signal):
    """ An implementation of nonlinear dynamics adaptation using Nengo,
    as described in (DeWolf, Stewart, Slotine, and Eliasmith, 2016)

    The model learns to account for unknown / unmodelled external or
    internal forces, using an efferent copy of the control signal to train.

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
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
    intercepts : list of two floats, optional (Default: (0.5, 1.0))
        voltage threshold range for neurons
    spiking : boolean, optional (Default: False)
        use spiking or rate mode neurons
    weights_file : string, optional (Default: None)
        path to file where learned weights are saved
    backend : string, optional (Default: nengo)
        {'nengo', 'nengo_ocl', 'nengo_spinnaker'}
    session: int, optional (Default: None)
        if doing multiple sessions of n runs to average over.
        if set to None it will search for the most recent session
        to save to, saving to 'session0' if none exist
    run: int, optional (Default: None)
        the current run number. This value will automatically
        increment based on the last run saved in the test_name
        folder. The user can specify a run if they desire to
        overwrite a previous run. If set to None then the test_name
        folder will be searched for the next run to save as
    test_name: string, optional (Default: 'test')
        the test name to save the weights under
    autoload: boolean, optional (Default: False)
        set true if you would like the script to check the 'test_name'
        directory for the most recent saved weights, False to start
        with a zeros array of weights, or a specified 'weights_file'
    function: function optional (Default: None)
        the function that nengo will try to approximate to bootstrap learning.
        This provides nengo with a starting point for adaptation instead of
        learning from zeros
    send_redis_spikes: boolean, optional (Default: False)
        True to send spiking information to redis for display purposes (mainly
        used with vrep_display.py)
    probe_weights: boolean, optional (Default: False)
        True to get decoders
    debug_print: boolean optional (Default: False)
        True to display debug print statements
    neuron_type: string: optional (Default: 'lif')
        the neuron model to use
    """

    def __init__(self, n_input, n_output, n_neurons=1000, n_ensembles=1,
                 seed=None, pes_learning_rate=1e-6, intercepts_bounds=(-0.9, 0.0),
                 intercepts_mode=-0.9, weights_file=None, backend='nengo',
                 function=None, send_redis_spikes=False, encoders=None,
                 probe_weights=False, debug_print=False, neuron_type='lif',
                 intercepts=None, **kwargs):

        if backend == 'nengo_cpu':
            backend = 'nengo'
        if backend == 'nengo_gpu':
            backend = 'nengo_ocl'

        """ Create adaptive population with the provided parameters"""
        self.tau_output = 0.2
        self.tau_training = 0.012
        self.tau_input = 0.012
        self.n_input = n_input
        self.n_output = n_output
        self.n_neurons = n_neurons
        self.n_ensembles = n_ensembles
        self.neuron_type = neuron_type
        self.seed = seed
        self.pes_learning_rate = pes_learning_rate
        self.intercepts_bounds = intercepts_bounds
        self.intercepts_mode = intercepts_mode
        self.intercepts = intercepts
        self.weights_file = weights_file
        self.backend = backend
        self.function = function
        self.send_redis_spikes = send_redis_spikes
        self.encoders = encoders
        self.probe_weights = probe_weights
        self.zero_weights = False

        self.input_signal = np.zeros(self.n_input)
        self.training_signal = np.zeros(self.n_output)
        self.output = np.zeros(self.n_output)

        if self.weights_file is None:
            self.weights_file = ''

        self.nengo_model = nengo.Network(seed=self.seed)

        # Set the nerual model to use
        # if not isinstance(self.neuron_type, list):
        #     self.neuron_type = self.neuron_type.tolist()
        self.nengo_model.config[nengo.Ensemble].neuron_type = nengo.LIF()

        with self.nengo_model:

            def input_signals_func(t):
                """ Get the input and -1 * training signal """
                return self.input_signal
            input_signals = nengo.Node(
                input_signals_func, size_out=self.n_input)

            def training_signals_func(t):
                time.sleep(0.0001)
                return -self.training_signal
            training_signals = nengo.Node(
                training_signals_func, size_out=self.n_output)

            # make the adaptive population output accessible
            def output_func(t, x):
                """ stores the adaptive output for use in control() """
                self.output = np.copy(x)
            output = nengo.Node(output_func, size_in=self.n_output, size_out=0)

            # specify intercepts such that neurons are
            # active throughout the whole range specified


            if intercepts is None:
                self.intercepts = AreaIntercepts(
                    dimensions=self.n_input,
                    base=Triangular(self.intercepts_bounds[0],
                        self.intercepts_mode, self.intercepts_bounds[1]))
            else:
                self.intercepts = intercepts

            self.adapt_ens = []
            self.conn_learn = []
            for ii in range(self.n_ensembles):
                self.adapt_ens.append(nengo.Ensemble(n_neurons=self.n_neurons,
                                           dimensions=self.n_input,
                                           intercepts=self.intercepts,
                                           radius=np.sqrt(self.n_input),
                                           **kwargs))
                if ii == 0 and debug_print:
                    print('==========\n %i Ensembles' % self.n_ensembles)

                try:
                    # if the NengoLib is installed, use it
                    # to optimize encoder placement
                    import nengolib
                    if self.encoders is None:
                        self.adapt_ens[ii].encoders = (
                            nengolib.stats.ScatteredHypersphere(surface=True))
                        if ii==0 and debug_print:
                            print('NengoLib used to optimize encoders placement')
                    else:
                        self.adapt_ens[ii].encoders = self.encoders[ii]
                        if ii==0 and debug_print:
                            print('Using user defined encoder values')
                except ImportError:
                    if ii==0:
                        print('Nengo lib not installed, encoder ' +
                              'placement will be sub-optimal.')

                # hook up input signal to adaptive population to provide context
                nengo.Connection(
                    input_signals,
                    self.adapt_ens[ii],
                    synapse=self.tau_input,
                    )

                # load weights from file if they exist, otherwise use zeros
                if self.weights_file == '~':
                    self.weights_file = os.path.expanduser(self.weights_file)
                if ii==0 and debug_print:
                    print('Backend: ', self.backend)
                    print('Shape of weights file: ',
                            np.array(self.weights_file).shape)
                    print('Weights file: %s' % self.weights_file)
                if not os.path.isfile('%s' % self.weights_file):
                    if not isinstance(self.weights_file, dict):
                        if ii==0 and debug_print:
                            print('No weights found, starting with zeros')
                        transform = np.zeros((self.n_output,
                            self.adapt_ens[ii].n_neurons))
                        self.zero_weights = True

                # set up learning connections
                if not self.zero_weights:

                    # if passing an npz file
                    if os.path.isfile('%s' % self.weights_file):
                        loaded_weights = np.load(self.weights_file)['weights']
                    # if passing the dictionary in directly
                    elif isinstance(self.weights_file, dict):
                        loaded_weights = self.weights_file['weights']
                    else:
                        loaded_weights = self.weights_file
                    # select sub-weights if network broken up into multiple
                    # ensembles
                    if self.n_ensembles ==1:
                        transform = loaded_weights
                    else:
                        transform = loaded_weights[ii]
                    # remove third dimension if present
                    if len(transform.shape) > 2:
                        transform = np.squeeze(transform)
                    if ii == 0 and debug_print:
                        print('Loading weights: \n', transform)
                        print('Loaded weights all zeros: ',
                              np.allclose(transform, 0))
                        print('Transform', transform)

                self.conn_learn.append(
                    nengo.Connection(
                        self.adapt_ens[ii].neurons,
                        output,
                        learning_rule_type=(None if self.pes_learning_rate == 0
                            else nengo.PES(self.pes_learning_rate)),
                        transform=transform,
                        synapse=self.tau_output,
                        )
                    )

                if ii==0 and debug_print:
                    print('==========')

                # hook up the training signal to the learning rule
                if self.pes_learning_rate != 0:
                    nengo.Connection(
                        training_signals,
                        self.conn_learn[ii].learning_rule,
                        synapse=self.tau_training,
                        )

            if self.backend == 'nengo' and self.probe_weights:
                self.nengo_model.weights_probe = nengo.Probe(self.conn_learn[0],
                        'weights', synapse=None)


        self.sim = nengo.Simulator(self.nengo_model, dt=.001,
                                   progress_bar=False)

    @property
    def params(self):
        params = {'source': 'dynamics_adaptation',
                  'n_input': self.n_input,
                  'n_output': self.n_output,
                  'n_neurons': self.n_neurons,
                  'n_ensembles': self.n_ensembles,
                  'neuron_type': self.neuron_type,
                  'seed': self.seed,
                  'pes': self.pes_learning_rate,
                  'intercepts': self.intercepts,
                  # 'intercepts_bounds': self.intercepts_bounds,
                  # 'intercepts_mode': self.intercepts_mode,
                  'backend': self.backend,
                  #'function': self.function,
                  'send_redis_spikes': self.send_redis_spikes,
                  'encoders': self.encoders,
                  'probe_weights': self.probe_weights,
                  'zero_weights': self.zero_weights,
                  'tau_input': self.tau_input,
                  'tau_output': self.tau_output,
                  'tau_training': self.tau_training}
        return params

    def return_params(self):
        """
        Returns a dictionary of the function parameters

        This is used for tracking parameters during testing and saving to
        a database
        """

        return self.params

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
        if self.backend == 'nengo' or self.backend == 'nengo_ocl':
            self.sim.run(time_in_seconds=.001, progress_bar=False)
        elif self.backend == 'nengo_spinnaker':
            # update spinnaker inputs
            self.sim.async_update()
        else:
            self.sim.run(time_in_seconds=.001)

        return self.output

    def get_weights(self, backend=None):
        """ Get the current weights

        backend: string: Optional, (Default: None)
            'nengo': use nengo as the backend for the adaptive population
            'nengo_spinnaker': use spinnaker as the adaptive population
        """

        if backend is not None:
            # if a backend is passed in, overwrite the current self.backend
            self.backend = backend

        if self.backend == 'nengo_spinnaker':
            import nengo_spinnaker.utils.learning
            # Need to power cycle spinnaker for repeated runs, so sim.close and
            # sleep is unnecessary, unless power cycling is automated using two
            # ethernet connections, then can uncomment the following two lines
            #self.sim.close()
            #time.sleep(5)
            weights=([nengo_spinnaker.utils.learning.get_learnt_decoders(
                     self.sim, ens) for ens in self.adapt_ens])

        else:
            weights=([self.sim.signals[self.sim.model.sig[conn]['weights']]
                     for conn in self.conn_learn])

        return weights

class DummySolver(nengo.solvers.Solver):
    """ A Nengo weights solver that returns a provided set of weights.
    """
    def __init__(self, fixed):
        self.fixed = fixed
        self.weights = False

    def __call__(self, A, Y, rng=None, E=None):
        return self.fixed, {}


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
