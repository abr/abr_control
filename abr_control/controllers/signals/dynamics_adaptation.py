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
from abr_control.utils.paths import cache_dir
from .signal import Signal

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
                 seed=None, pes_learning_rate=1e-6, intercepts=(-0.9, 0.0),
                 intercepts_mode=-0.9, weights_file=None, backend='nengo',
                 function=None, send_redis_spikes=False, encoders=None,
                 probe_weights=False, debug_print=False, neuron_type='lif',
                 **kwargs):

        if backend == 'nengo_cpu':
            backend == 'nengo'
        if backend == 'nengo_gpu':
            backend == 'nengo_ocl'

        """ Create adaptive population with the provided parameters"""
        self.n_input = n_input
        self.n_output = n_output
        self.n_neurons = n_neurons
        self.n_ensembles = n_ensembles
        self.neuron_type = neuron_type
        self.seed = seed
        self.pes_learning_rate = pes_learning_rate
        self.intercepts_bounds = intercepts
        self.intercepts_mode = intercepts_mode
        self.weights_file = weights_file
        self.backend = backend
        self.function = function
        self.send_redis_spikes = send_redis_spikes
        self.encoders = encoders
        self.probe_weights = probe_weights

        self.input_signal = np.zeros(self.n_input)
        self.training_signal = np.zeros(self.n_output)
        self.output = np.zeros(self.n_output)

        if self.weights_file is None:
            self.weights_file = ''

        self.nengo_model = nengo.Network(seed=self.seed)
        self.nengo_model.config[nengo.Connection].synapse = None

        # Set the nerual model to use
        if self.neuron_type.lower() == 'lif':
            self.nengo_model.config[nengo.Ensemble].neuron_type = nengo.LIF()

        elif self.neuron_type.lower() == 'relu':
            if backend == 'nengo_spinnaker':
                print('Spinnaker can only use LIF neuron models')
                self.nengo_model.config[nengo.Ensemble].neuron_type = nengo.LIF()
            else:
                self.nengo_model.config[nengo.Ensemble].neuron_type = nengo.RectifiedLinear()

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
            self.intercepts = AreaIntercepts(
                dimensions=self.n_input,
                base=Triangular(self.intercepts_bounds[0],
                    self.intercepts_mode, self.intercepts_bounds[1]))

            self.adapt_ens = []
            self.conn_learn = []

            for ii in range(self.n_ensembles):
                self.adapt_ens.append(nengo.Ensemble(n_neurons=self.n_neurons,
                                           dimensions=self.n_input,
                                           intercepts=self.intercepts,
                                           radius=np.sqrt(self.n_input),
                                           **kwargs))
                print('*** ENSEMBLE %i ***' % ii)

                try:
                    # if the NengoLib is installed, use it
                    # to optimize encoder placement
                    import nengolib
                    if self.encoders is None:
                        self.adapt_ens[ii].encoders = (
                            nengolib.stats.ScatteredHypersphere(surface=True))
                        print('NengoLib used to optimize encoders placement')
                    else:
                        self.adapt_ens[ii].encoders = self.encoders
                        print('Using user defined encoder values')
                except ImportError:
                    print('Nengo lib not installed, encoder ' +
                          'placement will be sub-optimal.')

                # hook up input signal to adaptive population to provide context
                nengo.Connection(input_signals, self.adapt_ens[ii])

                # load weights from file if they exist, otherwise use zeros
                if self.weights_file == '~':
                    self.weights_file = os.path.expanduser(self.weights_file)
                print('Backend: ', self.backend)
                print('Weights file: %s' % self.weights_file)
                if not os.path.isfile('%s' % self.weights_file):
                    print('No weights found, starting with zeros')
                    transform = np.zeros((self.n_output,
                        self.adapt_ens[ii].n_neurons))

                # set up learning connections
                if self.function is not None and (self.weights_file is None or
                        self.weights_file is ''):
                    print("Using provided function to bootstrap learning")
                    eval_points = Concatenate([nengo.dists.Choice([0]), nengo.dists.Choice([0]),
                        nengo.dists.Uniform(-1, 1), nengo.dists.Uniform(-1, 1)])
                    self.conn_learn.append(nengo.Connection(
                                           self.adapt_ens[ii], output,
                                           learning_rule_type=nengo.PES(
                                               self.pes_learning_rate),
                                           function=self.function,
                                           eval_points=eval_points))
                else:
                    if self.backend == 'nengo_spinnaker':
                        if os.path.isfile('%s' % self.weights_file):
                            if self.n_ensembles == 1:
                                transform = np.squeeze(np.load(self.weights_file)['weights']).T
                            else:
                                transform = np.squeeze(np.load(self.weights_file)['weights'])[ii].T
                            print('Loading weights: \n', transform)
                            print('Loaded weights all zeros: ', np.allclose(transform, 0))



                        print(transform.T.shape)

                        self.conn_learn.append(nengo.Connection(
                                               self.adapt_ens[ii], output,
                                               function=lambda x:
                                                   np.zeros(self.n_output),
                                               learning_rule_type=nengo.PES(
                                                 self.pes_learning_rate),
                                               solver=DummySolver(transform.T)))
                    else:

                        if os.path.isfile('%s' % self.weights_file):
                            if self.n_ensembles ==1:
                                transform = np.load(self.weights_file)['weights']
                            else:
                                transform = np.load(self.weights_file)['weights'][ii]
                            # remove third dimension if present
                            if len(transform.shape) > 2:
                                transform = np.squeeze(transform)
                            print('Loading weights: \n', transform)
                            print('Loaded weights all zeros: ', np.allclose(transform, 0))

                        self.conn_learn.append(nengo.Connection(
                                               self.adapt_ens[ii].neurons, output,
                                               learning_rule_type=nengo.PES(
                                                 self.pes_learning_rate),
                                               transform=transform))

                # hook up the training signal to the learning rule
                nengo.Connection(
                    training_signals, self.conn_learn[ii].learning_rule,
                    synapse=None)




            if self.backend != 'nengo_spinnaker' and self.send_redis_spikes:
                # Send spikes via redis
                def send_spikes(t, x):
                        v = np.where(x != 0)[0]
                        if len(v) > 0:
                            msg = struct.pack('%dI' % len(v), *v)
                        else:
                            msg = ''
                        r.set('spikes', msg)
                        self.activity = x
                source_node = nengo.Node(send_spikes, size_in=self.n_neurons)
                nengo.Connection(
                    self.adapt_ens[0].neurons[:self.n_neurons],
                    source_node,
                    synapse=None)
            def save_x(t, x):
                self.x = x
            x_node = nengo.Node(save_x, size_in=self.n_input)
            nengo.Connection(self.adapt_ens[0], x_node, synapse=None)

            if self.backend == 'nengo' and self.probe_weights:
                self.nengo_model.weights_probe = nengo.Probe(self.conn_learn[0], 'weights', synapse=None)





        #nengo.cache.DecoderCache().invalidate()
        if self.backend == 'nengo':
            self.sim = nengo.Simulator(self.nengo_model, dt=.001)
        elif self.backend == 'nengo_ocl':
            try:
                import nengo_ocl
            except ImportError:
                raise Exception('Nengo OCL not installed, ' +
                                'cannot use this backend.')
            import pyopencl as cl
            # Here, the context would be to use all devices from platform [0]
            ctx = cl.Context(cl.get_platforms()[0].get_devices())
            self.sim = nengo_ocl.Simulator(self.nengo_model, context=ctx, dt=.001)
        elif self.backend == 'nengo_spinnaker':
            try:
                import nengo_spinnaker
            except ImportError:
                raise Exception('Nengo SpiNNaker not installed, ' +
                                'cannot use this backend.')
            if debug_print:
                # turn on debug printing
                logging.basicConfig(level=logging.DEBUG)

            self.sim = nengo_spinnaker.Simulator(self.nengo_model)
            # start running the spinnaker model
            self.sim.async_run_forever()

        else:
            raise Exception('Invalid backend specified')

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
                  'intercepts_bounds': self.intercepts_bounds,
                  'intercepts_mode': self.intercepts_mode,
                  'backend': self.backend,
                  #'function': self.function,
                  'send_redis_spikes': self.send_redis_spikes,
                  'encoders': self.encoders,
                  'probe_weights': self.probe_weights}
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
