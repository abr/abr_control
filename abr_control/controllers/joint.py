import numpy as np


class controller:
    """ Implements a joint controller
    """

    def __init__(self, robot_config, kp=1, kv=None):

        self.robot_config = robot_config

        # proportional gain term
        self.kp = kp
        # derivative gain term
        self.kv = np.sqrt(self.kp) if kv is None else kv

    def control(self, q, dq, target_pos, target_vel=None):
        """Generate a control signal to move the arm through
           joint space to the desired joint angle position

        q np.array: current joint angles
        dq np.array: current joint velocities
        target_pos np.array: desired joint angles
        target_vel np.array: desired joint velocities
        """

        q_tilde = ((target_pos - q + np.pi) % (np.pi * 2)) - np.pi
        if target_vel is None:
            target_vel = np.zeros(self.robot_config.num_joints)

        # get the joint space inertia matrix
        Mq = self.robot_config.Mq(q)
        # get the gravity compensation signal
        Mq_g = self.robot_config.Mq_g(q)

        # calculated desired joint control signal
        u = - Mq_g + np.dot(Mq, (self.kp * q_tilde +
                                 self.kv * (target_vel - dq)))

        return u
