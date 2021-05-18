import numpy as np

from .controller import Controller
from abr_control.utils import transformations
from abr_control.utils.transformations import quaternion_multiply, quaternion_conjugate


class Joint(Controller):
    """Implements a joint space controller

    Moves the arm joints to a set of specified target angles

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    kp : float, optional (Default: 1)
        proportional gain term
    kv : float, optional (Default: None)
        derivative gain term, a good starting point is sqrt(kp)
    quaternions : list, optional (Default: None)
        a boolean list of which joints are quaternions
    """

    def __init__(self, robot_config, kp=1, kv=None, quaternions=None,
                 account_for_gravity=True):
        super().__init__(robot_config)

        self.kp = kp
        self.kv = np.sqrt(self.kp) if kv is None else kv
        self.account_for_gravity = account_for_gravity
        self.ZEROS_N_JOINTS = np.zeros(robot_config.N_JOINTS)

        if quaternions is not None:
            self.quaternions = quaternions
            self.q_tilde = self.q_tilde_quat
        else:
            self.q_tilde = self.q_tilde_angle

    def q_tilde_angle(self, q, target):
        # calculate the direction for each joint to move, wrapping
        # around the -pi to pi limits to find the shortest distance
        q_tilde = ((target - q + np.pi) % (np.pi * 2)) - np.pi
        return q_tilde

    def q_tilde_quat(self, q, target):
        """Compute the error signal when there are quaternions in the state
        space and target signals. If there are quaternions in the state space,
        calculate the error and then transform them to 3D space for the control
        signal.

        NOTE: Assumes that for every quaternion there are 3 motors, that effect
        movement along the x, y, and z axes, applied in that order.

        Parameters
        ----------
        q : float numpy.array
            mix of joint angles and quaternions [quaternions & radians]
        target : float numpy.array
            mix of target joint angles and quaternions [quaternions & radians]
        """
        # for each quaternion in the list, the output size is reduced by 1
        q_tilde = np.zeros(len(q) - np.sum(self.quaternions))

        q_index = 0
        q_tilde_index = 0

        for quat_bool in self.quaternions:
            if quat_bool:
                # if the joint position is a quaternion, calculate error
                joint_q = q[q_index:q_index+4]
                error = quaternion_multiply(
                    target[q_index:q_index+4],
                    quaternion_conjugate(joint_q),
                )

                # NOTE: we need to rotate this back to undo the current angle
                # because the actuators don't rotate with the ball joint
                u = quaternion_multiply(
                    quaternion_conjugate(joint_q),
                    quaternion_multiply(
                        error,
                        joint_q,
                    )
                )

                # convert from quaternion to 3D angular forces to apply
                # https://studywolf.wordpress.com/2018/12/03/force-control-of-task-space-orientation/
                q_tilde[q_tilde_index:q_tilde_index+3] = u[1:] * np.sign(u[0])

                q_index += 4
                q_tilde_index += 3
            else:
                q_tilde[q_tilde_index] = self.q_tilde_angle(
                    q[q_index], target[q_index])

                q_index += 1
                q_tilde_index += 1

        return q_tilde

    def generate(self, q, dq, target, target_velocity=None):
        """Generate a joint space control signal

        Parameters
        ----------
        q : float numpy.array
            current joint angles [radians]
        dq : float numpy.array
            current joint velocities [radians/second]
        target : float numpy.array
            desired joint angles [radians]
        target_velocity : float numpy.array, optional (Default: None)
            desired joint velocities [radians/sec]
        """

        if target_velocity is None:
            target_velocity = self.ZEROS_N_JOINTS

        q_tilde = self.q_tilde(q, target)

        # get the joint space inertia matrix
        M = self.robot_config.M(q)
        u = np.dot(M, (self.kp * q_tilde + self.kv * (target_velocity - dq)))
        # account for gravity
        if self.account_for_gravity:
            u -= self.robot_config.g(q)

        return u
