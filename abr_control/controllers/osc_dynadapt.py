import numpy as np

try:
    import nengo
except ImportError:
    print('Nengo module needs to be installed to use this controller.')

nengo_ocl = None
# try:
#     import nengo_ocl
# except ImportError:
#     print('Nengo OCL not installed, simulation will be slower.')

from . import osc
from .keeplearningsolver import KeepLearningSolver


class controller(osc.controller):
    """ Extension of the OSC controller that incorporates dynamics
    adaptation using a Nengo model
    """

    def __init__(self, robot_config,
                 pes_learning_rate=1e-6, voja_learning_rate=1e-6,
                 weights_file=None, encoders_file=None, **kwargs):
        """
        pes_learning_rate float: controls the speed of neural adaptation
                                 for training the dynamics compensation term
        voja_learning_rate float: controls the speed of neural adaptation
                                  for shifting the sensitivity of the CB
                                  encoders towards areas most often explored
        weights_file string: path to file where learned weights are saved
        encoders_file string: path to file where learned encoders are saved
        """

        super(controller, self).__init__(robot_config, **kwargs)

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

            adapt_ens = nengo.Ensemble(
                seed=10,
                n_neurons=2000,
                dimensions=self.robot_config.num_joints * 2,
                radius=np.sqrt(self.robot_config.num_joints * 2) * 10)

            if encoders_file is not None:
                try:
                    encoders = np.load(encoders_file)['encoders'][-1]
                    adapt_ens.encoders = encoders
                    print('Loaded encoders from %s' % encoders_file)
                except Exception:
                    print('No encoders file found, generating normally')
                    pass

            # connect input to CB with Voja so that encoders shift to
            # most commonly explored areas of state space
            conn_in = nengo.Connection(
                qdq_input,
                adapt_ens,
                learning_rule_type=nengo.Voja(voja_learning_rate))

            conn_learn = \
                nengo.Connection(
                    adapt_ens, output,
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
            self.probe_encoders = nengo.Probe(conn_in.learning_rule, 'scaled_encoders')

        if nengo_ocl is not None:
            self.sim = nengo_ocl.Simulator(nengo_model, dt=.001)
        else:
            self.sim = nengo.Simulator(nengo_model, dt=.001)

    def control(self, q, dq, target_xyz):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_xyz np.array: the current target for the end-effector
        """

        # store local copies to feed in to the adaptive population
        self.q = q
        self.dq = dq

        # generate the osc signal
        u = super(controller, self).control(q, dq, target_xyz)

        # run the simulation to generate the adaptive signal
        self.sim.run(time_in_seconds=.001, progress_bar=False)

        u += self.u_adapt
        return u
