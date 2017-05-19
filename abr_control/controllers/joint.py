import numpy as np

from . import controller

class Joint(controller.Controller):
    """ Implements a joint controller
    """

    def __init__(self, robot_config, kp=1, kv=None):
        self.robot_config = robot_config
        super(Joint,self).__init__(robot_config=self.robot_config)
        # proportional gain term
        self.kp = kp
        # derivative gain term
        self.kv = np.sqrt(self.kp) if kv is None else kv
        self.ZEROS_NUM_JOINTS = np.zeros(robot_config.NUM_JOINTS)
        self.q_tilde = np.copy(self.zeros_NUM_JOINTS)

    def generate(self, q, dq, target_pos, target_vel=None):
        """Generate a control signal to move the arm through
           joint space to the desired joint angle position

        q np.array: current joint angles
        dq np.array: current joint velocities
        target_pos np.array: desired joint angles
        target_vel np.array: desired joint velocities
        """

        self.q_tilde = ((target_pos - q + np.pi) % (np.pi * 2)) - np.pi
        if target_vel is None:
            target_vel = self.ZEROS_NUM_JOINTS
        # TODO: do we need the M term here?
        # get the joint space inertia matrix
        # M = self.robot_config.M(q)
        # get the gravity compensation signal
        g = self.robot_config.g(q)

        # calculated desired joint control signal
        # self.training_signal = np.dot(M, (self.kp * self.q_tilde +
        #                               self.kv * (target_vel - dq)))
        self.training_signal = self.kp * self.q_tilde - self.kv * dq
        u = self.training_signal - g

        return u
