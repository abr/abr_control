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
        self.zeros_num_joints = np.zeros(robot_config.num_joints)
        self.q_tilde = np.copy(self.zeros_num_joints)

    def control(self, q, dq, target_pos, target_vel=None):
        """Generate a control signal to move the arm through
           joint space to the desired joint angle position

        q np.array: current joint angles
        dq np.array: current joint velocities
        target_pos np.array: desired joint angles
        target_vel np.array: desired joint velocities
        """

        self.q_tilde = ((target_pos - q + np.pi) % (np.pi * 2)) - np.pi
        if target_vel is None:
            target_vel = self.zeros_num_joints

        # get the joint space inertia matrix
        M = self.robot_config.M(q)
        # get the gravity compensation signal
        g = self.robot_config.g(q)

        # calculated desired joint control signal
        self.training_signal = np.dot(M, (self.kp * self.q_tilde +
                                      self.kv * (target_vel - dq)))
        # self.training_signal = self.kp * self.q_tilde - self.kv * dq
        u = self.training_signal - g
        print('u: ', [float('%.3f' % val) for val in u])

        return u
