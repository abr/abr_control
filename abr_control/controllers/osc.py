import numpy as np

from . import controller


class OSC(controller.Controller):
    """ Implements an operational space controller (OSC)

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    kp : float, optional (Default: 1)
        proportional gain term
    kv : float, optional (Default: None)
        derivative gain term, a good starting point is sqrt(kp)
    vmax : float, optional (Default: 0.5)
        The max allowed velocity of the end-effector [meters/second].
        If the control signal specifies something above this
        value it is clipped, if set to None no clipping occurs
    null_control : boolean, optional (Default: True)
        Apply a secondary control signal which
        drives the arm to specified resting joint angles without
        affecting the movement of the end-effector
    use_g : boolean, optional (Default: True)
        calculate and compensate for the effects of gravity
    use_C : boolean, optional (Default: False)
        calculate and compensate for the Coriolis and
        centripetal effects of the arm
    use_dJ : boolean, optional (Default: False)
        use the Jacobian derivative wrt time

    Attributes
    ----------
    nkp : float
        proportional gain term for null controller
    nkv : float
        derivative gain term for null controller
    """
    def __init__(self, robot_config, kp=1, kv=None, vmax=0.5,
                 null_control=True, use_g=True, use_C=False, use_dJ=False):

        super(OSC, self).__init__(robot_config)

        self.kp = kp
        self.kv = np.sqrt(self.kp) if kv is None else kv
        self.vmax = vmax
        self.lamb = self.kp / self.kv
        self.null_control = null_control
        self.use_g = use_g
        self.use_C = use_C
        self.use_dJ = use_dJ

        # null_indices is a mask for identifying which joints have REST_ANGLES
        self.null_indices = ~np.isnan(self.robot_config.REST_ANGLES)
        self.dq_des = np.zeros(self.robot_config.N_JOINTS)
        self.IDENTITY_N_JOINTS = np.eye(self.robot_config.N_JOINTS)
        # null space filter gains
        self.nkp = self.kp * .1
        self.nkv = np.sqrt(self.nkp)

    def generate(self, q, dq,
                 target_pos, target_vel=np.zeros(3),
                 target_quat=None, target_w=np.zeros(3),
                 ref_frame='EE', offset=[0, 0, 0]):
        """ Generates the control signal to move the EE to a target

        Parameters
        ----------
        q : float numpy.array
            current joint angles [radians]
        dq : float numpy.array
            current joint velocities [radians/second]
        target_pos : float numpy.array
            desired joint angles [radians]
        target_vel : float numpy.array, optional (Default: numpy.zeros)
            desired joint velocities [radians/sec]
        target_quat : float numpy.array, optional Default: None)
            the target orientation as a quaternion in the form [w, x, y, z]
        target_w : float numpy.array, optional (Default: numpy.zeros)
            the target angular velocities
        ref_frame : string, optional (Default: 'EE')
            the point being controlled, default is the end-effector.
        offset : list, optional (Default: [0, 0, 0])
            point of interest inside the frame of reference [meters]
        """
        # calculate the end-effector position information
        xyz = self.robot_config.Tx(ref_frame, q, x=offset)

        # calculate the Jacobian for the end effector
        J = self.robot_config.J(ref_frame, q, x=offset)
        # isolate position component of Jacobian
        J = J[:3]
        dx = np.dot(J, dq)

        # calculate the inertia matrix in joint space
        M = self.robot_config.M(q)

        # calculate the inertia matrix in task space
        M_inv = np.linalg.inv(M)
        # calculate the Jacobian for end-effector with no offset
        JEE = self.robot_config.J(ref_frame, q)[:3]
        Mx_inv = np.dot(JEE, np.dot(M_inv, JEE.T))
        # using the rcond to set singular values < thresh to 0
        # is slightly faster than doing it manually with svd
        # singular values < (rcond * max(singular_values)) set to 0
        Mx = np.linalg.pinv(Mx_inv, rcond=.01)

        u_task = np.zeros(3)  # task space control signal

        # calculate the position error
        x_tilde = np.array(xyz - target_pos)

        if self.vmax is not None:
            # implement velocity limiting
            sat = self.vmax / (self.lamb * np.abs(x_tilde))
            if np.any(sat < 1):
                index = np.argmin(sat)
                unclipped = self.kp * x_tilde[index]
                clipped = self.kv * self.vmax * np.sign(x_tilde[index])
                scale = np.ones(3, dtype='float32') * clipped / unclipped
                scale[index] = 1
            else:
                scale = np.ones(3, dtype='float32')

            u_task[:3] = -self.kv * (dx - target_vel -
                                     np.clip(sat / scale, 0, 1) *
                                     -self.lamb * scale * x_tilde)
        else:
            # generate (x,y,z) force without velocity limiting)
            u_task[:3] = -self.kp * x_tilde

        # incorporate task space inertia matrix
        u = np.dot(J.T, np.dot(Mx, u_task))

        if self.vmax is None:
            u -= np.dot(M, dq)

        if self.use_dJ:
            # add in estimate of current acceleration
            dJ = self.robot_config.dJ(ref_frame, q=q, dq=dq)
            # apply mask
            dJ = dJ[:3]
            u -= np.dot(Mx, np.dot(dJ, dq))

        if self.use_C:
            # add in estimation of centripetal and Coriolis effects
            u -= self.robot_config.C(q=q, dq=dq)

        # TODO: self.training_signal is a hack, but keeps API
        # cleaner than returning something other than just u
        # NOTE: training signal should not include gravity compensation
        self.training_signal = np.copy(u)

        # cancel out effects of gravity
        if self.use_g:
            u -= self.robot_config.g(q=q)

        if self.null_control:
            # calculated desired joint angle acceleration using rest angles
            q_des = ((self.robot_config.REST_ANGLES - q + np.pi) %
                     (np.pi * 2) - np.pi)
            q_des[~self.null_indices] = 0.0
            self.dq_des[self.null_indices] = dq[self.null_indices]

            u_null = np.dot(M, (self.nkp * q_des - self.nkv * self.dq_des))

            Jbar = np.dot(M_inv, np.dot(JEE.T, Mx))
            null_filter = (self.IDENTITY_N_JOINTS - np.dot(J.T, Jbar.T))

            u += np.dot(null_filter, u_null)

        return u
