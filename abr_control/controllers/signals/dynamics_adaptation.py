import numpy as np
import os
import redis
import struct
import scipy
r = redis.StrictRedis(host='127.0.0.1')

try:
    import nengo
except ImportError:
    raise Exception('Nengo module needs to be installed to ' +
                    'use adaptive dynamics.')

nengolib = None
try:
    import nengolib
except ImportError:
    print('Nengo lib not installed, encoder placement will be sub-optimal.')


class AreaIntercepts(nengo.dists.Distribution):
    dimensions = nengo.params.NumberParam('dimensions')
    base = nengo.dists.DistributionParam('base')

    def __init__(self, dimensions, base=nengo.dists.Uniform(-1, 1)):
        super(AreaIntercepts, self).__init__()
        self.dimensions = dimensions
        self.base = base

    def __repr(self):
        return "AreaIntercepts(dimensions=%r, base=%r)" % (self.dimensions,
                                                           self.base)

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


class Signal():
    """ An implementation of dynamics adaptation using a Nengo model
    """

    def __init__(self, robot_config,
                 n_neurons=1000,
                 n_adapt_pop=1,
                 pes_learning_rate=1e-6,
                 intercepts=(0.5, 1.0),
                 spiking=False,
                 use_area_intercepts=True,
                 extra_dimension=False,
                 use_probes=False,
                 filter_error=False,
                 weights_file=None,
                 backend='nengo'):
        """
        n_neurons int: number of neurons per adaptive population
        n_adapt_pop: number of adaptive populations
        pes_learning_rate float: controls the speed of neural adaptation
                                 for training the dynamics compensation term
        intercepts list: voltage threshold range for neurons
        spiking boolean: use spiking or rate mode neurons
        use_area_intercepts boolean: set intercepts to be distributed or not
        extra_dimensions boolean: add an extra dimension for putting things
                                  on the hypersphere, or not
        use_probes boolean: record weights and spiking data or not
        filter_error boolean: apply low pass filter to the error signal or not
        weights_file string: path to file where learned weights are saved
        backend string: {'nengo', 'nengo_ocl', 'nengo_spinnaker'}
        """
        self.robot_config = robot_config

        self.u_adapt = np.zeros(self.robot_config.num_joints)

        weights_file = (['']*n_adapt_pop if
                        weights_file is None else weights_file)

        if use_probes:
            self.probe_weights = []
            # self.ens_activity = []
        else:
            self.probe_weights = None
            # self.ens_activity = None

        dim = self.robot_config.num_joints
        nengo_model = nengo.Network(seed=10)
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

            adapt_ens = []
            conn_learn = []
            for ii in range(n_adapt_pop):
                num_ens_dims = self.robot_config.num_joints * 2
                if extra_dimension:
                    num_ens_dims = self.robot_config.num_joints * 2 + 1

                intercepts = nengo.dists.Uniform(
                    intercepts[0], intercepts[1])
                if use_area_intercepts:
                    intercepts = AreaIntercepts(dimensions=num_ens_dims,
                                                base=intercepts)

                if spiking:
                    neuron_type = nengo.LIF()
                else:
                    neuron_type = nengo.LIFRate()

                adapt_ens.append(nengo.Ensemble(
                    n_neurons=n_neurons,
                    dimensions=num_ens_dims,
                    encoders=nengolib.stats.ScatteredHypersphere(
                        surface=True),
                    intercepts=intercepts,
                    neuron_type=neuron_type))

                def expanded_normalize(x):
                    if extra_dimension:
                        x = np.hstack([x, [1]])
                        x /= np.linalg.norm(x)
                    return x
                nengo.Connection(
                    qdq_input,
                    adapt_ens[ii][:num_ens_dims],
                    function=expanded_normalize,
                    synapse=0.005)

                # load weights from file if they exist, otherwise use zeros
                print ('%s' % weights_file[ii])
                if os.path.isfile('%s' % weights_file[ii]):
                    transform = np.load(weights_file[ii])['weights'][-1][0]
                    print('Loading transform: \n', transform)
                    print('Transform all zeros: ', np.allclose(transform, 0))
                else:
                    print('transform is zeros')
                    transform = np.zeros((adapt_ens[ii].n_neurons,
                                          self.robot_config.num_joints)).T

                conn_learn.append(
                    nengo.Connection(
                        adapt_ens[ii].neurons,
                        output,
                        learning_rule_type=nengo.PES(pes_learning_rate),
                        transform=transform))

                # Allow filtering of error signal
                def gate_error(x):
                    r.set('raw_error', x)
                    if filter_error:
                        if np.linalg.norm(x) > 2.0:
                            x /= np.linalg.norm(x) * 0.5
                    return x
                nengo.Connection(u_input, conn_learn[ii].learning_rule,
                                 # invert to provide error not reward
                                 transform=-1,
                                 function=gate_error,
                                 synapse=0.01)

                # Send spikes via redis
                def send_spikes(t, x):
                        v = np.where(x != 0)[0]
                        if len(v) > 0:
                            msg = struct.pack('%dI' % len(v), *v)
                        else:
                            msg = ''
                        r.set('spikes', msg)
                source_node = nengo.Node(send_spikes, size_in=25)
                nengo.Connection(
                    adapt_ens[ii].neurons[:25],
                    source_node,
                    synapse=None)

                if use_probes:
                    # record the weights once every second
                    self.probe_weights.append(nengo.Probe(
                        conn_learn[ii], 'weights', sample_every=1))

                    # record the activity to determine sparseness
                    # self.ens_activity = nengo.Probe(
                    #    adapt_ens[ii].neurons, sample_every=1)

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
        else:
            raise Exception('Invalid backend specified')
        self.backend = backend

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

        # run the simulation t generate the adaptive signal
        if self.backend == 'nengo' or self.backend == 'nengo_ocl':
            self.sim.run(time_in_seconds=.001, progress_bar=False)
        else:
            self.sim.run(time_in_seconds=.001)

        return self.u_adapt
