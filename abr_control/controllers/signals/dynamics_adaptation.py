import numpy as np

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

from abr_control.utils.keeplearningsolver import KeepLearningSolver


class Signal():
    """ An implementation of dynamics adaptation using a Nengo model
    """

    def __init__(self, robot_config,
                 n_neurons=1000,
                 n_adapt_pop=1,
                 pes_learning_rate=1e-6,
                 voja_learning_rate=1e-6,
                 weights_file=None,
                 backend='nengo'):
        """
        pes_learning_rate float: controls the speed of neural adaptation
                                 for training the dynamics compensation term
        voja_learning_rate float: controls the speed of neural adaptation
                                  for shifting the sensitivity of the CB
                                  encoders towards areas most often explored
        weights_file string: path to file where learned weights are saved
        """
        self.robot_config = robot_config

        self.u_adapt = np.zeros(self.robot_config.num_joints)

        weights_file = (['']*n_adapt_pop if
            weights_file is None else weights_file)

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

            adapt_ens=[]
            conn_learn=[]
            self.probe_weights=[]
            for ii in range(n_adapt_pop):
                adapt_ens.append(nengo.Ensemble(
                    n_neurons=n_neurons,
                    dimensions=self.robot_config.num_joints * 2,
                    encoders = nengolib.stats.ScatteredHypersphere(
                        surface=True),
                    eval_points = nengolib.stats.ScatteredHypersphere(
                        surface=False),
                    intercepts=nengo.dists.Uniform(-.1,1)))

                nengo.Connection(
                    qdq_input,
                    adapt_ens[ii][:self.robot_config.num_joints * 2],
                    function=lambda x: x / np.linalg.norm(x))

                conn_learn.append(
                    nengo.Connection(
                        adapt_ens[ii][:self.robot_config.num_joints * 2], output,
                        # start with outputting just zero
                        function=lambda x, ii=ii: np.zeros(dim),
                        learning_rule_type=nengo.PES(pes_learning_rate),
                        # use the weights solver that lets you keep
                        # learning from the what's saved to file
                        solver=KeepLearningSolver(filename=weights_file[ii])))
                nengo.Connection(u_input, conn_learn[ii].learning_rule,
                                # invert because we're providing error not reward
                                transform=-1, synapse=.01)

                self.probe_weights.append(nengo.Probe(conn_learn[ii], 'weights'))

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
