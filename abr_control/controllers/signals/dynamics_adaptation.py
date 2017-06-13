import numpy as np
import os
import scipy.special

import abr_control.utils.os_utils
from abr_control.utils.paths import cache_dir
from .signal import Signal

try:
    import nengo
except ImportError:
    raise Exception('Nengo module needs to be installed to ' +
                    'use adaptive dynamics.')


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
    n_neurons : int, optional (Default: 1000)
        number of neurons per adaptive population
    pes_learning_rate : float, optional (Default: 1e-6)
        controls the speed of neural adaptation
    intercepts list : list of two floats, optional (Default: (0.5, 1.0))
        voltage threshold range for neurons
    spiking : boolean, optional (Default: False)
        use spiking or rate mode neurons
    weights_file : string, optional (Default: None)
        path to file where learned weights are saved
    backend : string, optional (Default: nengo)
        {'nengo', 'nengo_ocl', 'nengo_spinnaker'}
    """

    def __init__(self, n_input, n_output, n_neurons=1000, seed=1,
                 pes_learning_rate=1e-6, intercepts=(0.5, 1.0),
                 weights_file=None, backend='nengo', **kwargs):

        self.input_signal = np.zeros(n_input)
        self.training_signal = np.zeros(n_output)
        self.output = np.zeros(n_output)

        if weights_file is None:
            weights_file = ['']

        nengo_model = nengo.Network(seed=seed)
        with nengo_model:

            def input_signals_func(t):
                """ Get the input and -1 * training signal """
                return np.hstack([self.input_signal, -self.training_signal])
            input_signals = nengo.Node(
                input_signals_func, size_out=n_input+n_output)

            # make the adaptive population output accessible
            def output_func(t, x):
                """ stores the adaptive output for use in control() """
                self.output = np.copy(x)
            output = nengo.Node(output_func, size_in=n_output, size_out=0)

            # specify intercepts such that neurons are
            # active throughout the whole range specified
            intercepts = AreaIntercepts(
                dimensions=n_input,
                base=nengo.dists.Uniform(intercepts[0], intercepts[1]))

            adapt_ens = nengo.Ensemble(n_neurons=n_neurons, dimensions=n_input,
                                       intercepts=intercepts, **kwargs)

            try:
                # if the NengoLib is installed, use it
                # to optimize encoder placement
                import nengolib
                adapt_ens.encoders = (
                    nengolib.stats.ScatteredHypersphere(surface=True))
                print('NengoLib used to optimize encoders placement')
            except ImportError:
                print('Nengo lib not installed, encoder ' +
                      'placement will be sub-optimal.')

            # hook up input signal to adaptive population to provide context
            nengo.Connection(input_signals[:n_input], adapt_ens, synapse=0.005)

            # load weights from file if they exist, otherwise use zeros
            if os.path.isfile('%s' % weights_file):
                transform = np.load(weights_file)['weights'][-1][0]
                print('Loading weights: \n', transform)
                print('Loaded weights all zeros: ', np.allclose(transform, 0))
            else:
                print('No weights found, starting with zeros')
                transform = np.zeros((adapt_ens.n_neurons, n_output)).T

            # set up learning connections
            if backend == 'nengo_spinnaker':
                conn_learn = nengo.Connection(
                    adapt_ens, output,
                    learning_rule_type=nengo.PES(pes_learning_rate),
                    solver=DummySolver(transform.T))
            else:
                conn_learn = nengo.Connection(
                    adapt_ens.neurons, output,
                    learning_rule_type=nengo.PES(pes_learning_rate),
                    transform=transform)

            # hook up the training signal to the learning rule
            nengo.Connection(
                input_signals[n_input:], conn_learn.learning_rule,
                synapse=0.01)

        nengo.cache.DecoderCache().invalidate()
        if backend == 'nengo':
            self.sim = nengo.Simulator(nengo_model, dt=.001)
        elif backend == 'nengo_ocl':
            try:
                import nengo_ocl
            except ImportError:
                raise Exception('Nengo OCL not installed, ' +
                                'cannot use this backend.')
            import pyopencl as cl
            # Here, the context would be to use all devices from platform [0]
            ctx = cl.Context(cl.get_platforms()[0].get_devices())
            self.sim = nengo_ocl.Simulator(nengo_model, context=ctx, dt=.001)
        elif backend == 'nengo_spinnaker':
            try:
                import nengo_spinnaker
            except ImportError:
                raise Exception('Nengo SpiNNaker not installed, ' +
                                'cannot use this backend.')
            self.sim = nengo_spinnaker.Simulator(nengo_model)
            # start running the spinnaker model
            self.sim.async_run_forever()
        else:
            raise Exception('Invalid backend specified')
        self.backend = backend

    def generate(self, input_signal, training_signal):
        """ Generates the control signal

        Parameters
        ----------
        q : numpy.array
            the current joint angles [radians]
        dq : numpy.array
            the current joint velocities [radians/second]
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

    def save_weights(self, weights, trial=None, run=None,
                     location='test'):
        """ Save the current weights to the specified location

        Saved weights for individual runs. A group of runs is
        classified as a trial. Multiple trials can then be used
        to average over a set of learned runs. If trial is set
        to 0 then it is assumed that there will only be one set
        of runs.
        
        Parameters
        ----------
        weights
        trial: int, optional (Default: None)
            if doing multiple trials of n runs to average over.
            if set to None will assume no averaging over trials
        run: int, optional (Default: None)
            the current run number. This value will automatically
            increment based on the last run saved in the location
            folder. The user can specify a run if they desire to
            overwrite a previous run
        location: string, optional (Default: 'test')
            the save location
        """
        
        location = cache_dir + '/saved_weights/' + location
        try to open location
        else print location does not exist, saving to default location

        if trial is not None:
            location += '/trial%i' % trial
        else:
            look in location for most recent trial
        if run is not None:
            location += '/run%i' 
        else:
            look in location for most recent run and save as next run
        save file in location

    def load_weights(self, trial, run, location='test'):
        """ load the last set of weights

        Checks location to load the most recent set of weights, unless
        otherwise specified in trial and run

        Parameters
        ----------
        trial: int, optional (Default: 0)
            The trial to load the weights from, if not specified will
            take the most recent weights from location
        run: int, optional (Default: 0)
            the run to load the weights from, if not specified will take
            the most recent weights from location
        location: string, optional (Default: 'test')
            the save location
        """
        location = cache_dir + '/saved_weights/' + location
        # same as for saving weights, except end with load instead of save
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
