import numpy as np
import time

try:
    import nengo
except ImportError:
    print('Nengo module needs to be installed to use this controller.')

from . import osc
from .keeplearningsolver import KeepLearningSolver


class controller(osc.controller):
    """ Extension of the OSC controller that incorporates dynamics
    adaptation using a Nengo model
    """

    def __init__(self, robot_config, learning_rate=1e-3):
        """
        learning_rate float: controls the speed of neural adaptation
        """

        super(controller, self).__init__(robot_config)

        self.u_adapt = np.zeros(self.robot_config.num_joints)

        dim = self.robot_config.num_joints
        nengo_model = nengo.Network()
        with nengo_model:

            def qdq_input(t):
                """ returns q and dq scaled and bias to
                be around -1 to 1 """
                return self.robot_config.scaledown(
                    'CB', np.hstack([self.q, self.dq]))
            qdq_input = nengo.Node(qdq_input, size_out=dim*2)

            def u_input(t):
                """ returns the control signal for training """
                print('self.u: ', self.u)
                return self.u
            u_input = nengo.Node(u_input, size_out=dim)

            def u_adapt_output(t, x):
                """ stores the adaptive output for use in control() """
                self.u_adapt = np.copy(x)
            output = nengo.Node(u_adapt_output, size_in=dim, size_out=0)

            adapt_ens = nengo.Ensemble(n_neurons=500,
                                       dimensions=dim*2,
                                       # TODO: investigate best radius size
                                       radius=np.sqrt(dim*2),
                                       seed=10)

            nengo.Connection(qdq_input, adapt_ens)
            learn_conn = \
                nengo.Connection(adapt_ens, output,
                                 # start with outputting just zero
                                 function=lambda x: np.zeros(dim),
                                 learning_rule_type=nengo.PES(learning_rate),
                                 # use the weights solver that lets you keep
                                 # learning from the what's saved to file
                                 solver=KeepLearningSolver('weights.npz'))
            nengo.Connection(u_input, learn_conn.learning_rule,
                             # invert because we're providing error not reward
                             transform=-1, synapse=.01)

            self.probe_weights = nengo.Probe(learn_conn, 'weights',
                                             sample_every=.1)  # in seconds

        self.sim = nengo.Simulator(nengo_model)

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
        self.u = super(controller, self).control(q, dq, target_xyz)
        # run the simulation to generate the adaptive signal
        self.sim.run(time_in_seconds=.001, progress_bar=False)

        self.u += self.u_adapt

        print('u_adapt: ', self.u_adapt)

        # return the sum of the two
        return self.u
