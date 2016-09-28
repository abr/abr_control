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

    def __init__(self, robot_config, pes_learning_rate=1e-6,
                 weights_file=None):
        """
        pes_learning_rate float: controls the speed of neural adaptation
                                 for training the dynamics compensation term
        weights_file string: path to file where learned weights are saved
        """

        super(controller, self).__init__(robot_config)

        self.u_adapt = np.zeros(self.robot_config.num_joints)

        dim = self.robot_config.num_joints
        nengo_model = nengo.Network()
        with nengo_model:

            def u_input(t):
                """ returns the control signal for training """
                return self.training_signal
            u_input = nengo.Node(u_input, size_out=dim)

            def kinadapt_output(t, x):
                global L
                L = np.copy(x)
            output = nengo.Node(output=kinadapt_output,
                                size_in=dim, size_out=0)

            adapt_ens = nengo.Ensemble(**self.robot_config.adapt)

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

        # temporarily set them here for testing
        L = np.array([0.0935, 0.13453, 0.4251,
                      0.12, 0.3921, 0.0935, 0.0935, 0.0935])

        # run the model, update the parameter estimations
        # self.sim.run(dt=.001)
        parameters = tuple(q) + tuple(L)
        print('parameters: ', parameters)

        # calculate position of the end-effector
        self.xyz = self.robot_config.T('EE', parameters)

        # calculate the Jacobian for the end effector
        JEE = self.robot_config.J('EE', parameters)

        # calculate the inertia matrix in joint space
        Mq = self.robot_config.Mq(parameters)

        # calculate the effect of gravity in joint space
        Mq_g = self.robot_config.Mq_g(parameters)

        # convert the mass compensation into end effector space
        Mx_inv = np.dot(JEE, np.dot(np.linalg.inv(Mq), JEE.T))
        svd_u, svd_s, svd_v = np.linalg.svd(Mx_inv)
        # cut off any singular values that could cause control problems
        singularity_thresh = .00025
        for i in range(len(svd_s)):
            svd_s[i] = 0 if svd_s[i] < singularity_thresh else \
                1./float(svd_s[i])
        # numpy returns U,S,V.T, so have to transpose both here
        Mx = np.dot(svd_v.T, np.dot(np.diag(svd_s), svd_u.T))

        # calculate desired force in (x,y,z) space
        u_xyz = np.dot(Mx, target_xyz - self.xyz)
        # transform into joint space, add vel compensation
        # u = (self.kp * np.dot(JEE.T, u_xyz) - np.dot(Mq, self.kv * dq) -
        #      Mq_g)
        self.training_signal = (self.kp * np.dot(JEE.T, u_xyz) -
                                np.dot(Mq, self.kv * dq))
        # add in gravity compensation, not included in training signal
        u = self.training_signal - Mq_g

        # calculate the null space filter
        Jdyn_inv = np.dot(Mx, np.dot(JEE, np.linalg.inv(Mq)))
        null_filter = (np.eye(self.robot_config.num_joints) -
                       np.dot(JEE.T, Jdyn_inv))

        # calculate q0 target angle relative to object to prevent
        # getting stuck trying to reach the object while moving sideways
        target_angle = np.arctan2(target_xyz[1], target_xyz[0])
        # q0_des = (((target_angle - q[0]) + np.pi) %
        #           (np.pi*2) - np.pi)
        q0_des = ((target_angle - q[0]) % np.pi)

        # calculated desired joint angle acceleration using rest angles
        q_des = (((self.robot_config.rest_angles - q) + np.pi) %
                 (np.pi*2) - np.pi)
        # set desired angle for q0 to be relative to target position
        q_des[0] = q0_des
        u_null = np.dot(Mq, (self.kp * q_des - self.kv * dq))
        # let it be anywhere within np.pi / 4 range of target angle
        if q_des[0] < np.pi / 8.0 and q_des[0] > -np.pi / 8.0:
            u_null[0] = 0.0

        u += np.dot(null_filter, u_null)

        return u
