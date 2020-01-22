import numpy as np

from .controller import Controller


class Sliding(Controller):
    """ Implements sliding control based on the description
    in (Slotine, 1987), default parameters from paper.

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    kd : float, optional (Default: 160)
        gain term
    lambda : float, optional (Default: 30)
        gain term
    cartesian : boolean, optional (Default: True)
        if True transforms control from Cartesian into joint space
        if False control assumed to be entirely in joint space

    """

    def __init__(self, robot_config, kd=160.0, lamb=30.0, cartesian=True):

        super(Sliding, self).__init__(robot_config)

        self.kd = kd
        self.lamb = lamb
        self.cartesian = cartesian

    def generate(
        self,
        q,
        dq,
        target,
        target_velocity=0,
        target_acc=0,
        ref_frame="EE",
        offset=None,
    ):
        """ Generates the control signal to move the EE to a target

        Parameters
        ----------
        q : float numpy.array
            current joint angles [radians]
        dq : float numpy.array
            current joint velocities [radians/second]
        target : float numpy.array
            desired joint angles [radians]
        target_velocity : float numpy.array, optional (Default: numpy.zeros)
            desired joint velocities [radians/sec]
        ref_frame : string, optional (Default: 'EE')
            the point being controlled, default is the end-effector.
        offset : list, optional (Default: None)
            point of interest inside the frame of reference [meters]
        """

        offset = self.offset_zeros if offset is None else offset

        if self.cartesian:
            # calculate the position Jacobian for the end effector
            J = self.robot_config.J(ref_frame, q, x=offset)[:3]

            # calculate the end-effector position information
            xyz = self.robot_config.Tx(ref_frame, q, x=offset)
            dxyz = np.dot(J, dq)

            J_inv = np.linalg.pinv(J)
            dJ = self.robot_config.dJ(ref_frame, q, dq, x=offset)[:3]

            dq_ref = np.dot(J_inv, target_velocity + self.lamb * (target - xyz))
            ddq_ref = np.dot(
                J_inv,
                target_acc + self.lamb * (target_velocity - dxyz) - np.dot(dJ, dq_ref),
            )
        else:
            q_tilde = q - target
            dq_tilde = dq - target_velocity
            dq_ref = target_velocity - self.lamb * q_tilde
            ddq_ref = target_acc - self.lamb * dq_tilde

        # store the control signal s for training in case
        # dynamics adaptation signal is being used
        self.s = dq - dq_ref

        # calculate the inertia matrix in joint space
        M = self.robot_config.M(q)
        # calculate the partial centrifugal and Coriolis effects
        C = self.robot_config.C(q=q, dq=dq)
        # calculate the effects of gravity
        g = self.robot_config.g(q=q)

        u = np.dot(M, ddq_ref) + np.dot(C, dq_ref) + g - self.kd * self.s

        return u
