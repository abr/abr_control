import numpy as np

class Signal():
    """ An implementation of dynamics adaptation using a Nengo model
    """

    def __init__(self, robot_config,
                 min_joint_angles, max_joint_angles,
                 max_torque=1000):
        """
        min_joint_angles np.array: the lower bound on joint angles (radians)
        max_joint_angles np.array: the upper bound on joint angles (radians)
        NOTE: use None as a placeholder for joints that have no limits
        gains np.array: default is .01
        """

        self.min_joint_angles = np.asarray(min_joint_angles, dtype='float32')
        self.max_joint_angles = np.asarray(max_joint_angles, dtype='float32')

        if (self.max_joint_angles.shape[0] != robot_config.num_joints or
                self.min_joint_angles.shape[0] != robot_config.num_joints):
            raise Exception('joint angles vector incorrect size')
        # find where there aren't limits
        self.no_limits_min = np.isnan(self.min_joint_angles)
        self.no_limits_max = np.isnan(self.max_joint_angles)

        self.robot_config = robot_config
        self.max_torque = max_torque


    def generate(self, q):
        """ Generates the control signal

        q np.array: the current joint angles
        """

        avoid_min = np.zeros(self.robot_config.num_joints)
        avoid_min = np.minimum(np.exp(-1.0/(self.min_joint_angles - q)), self.max_torque)
        avoid_min[(q - self.min_joint_angles) < 0] = self.max_torque
        avoid_min[self.no_limits_min] = 0.0

        avoid_max = np.zeros(self.robot_config.num_joints)
        avoid_max = -np.minimum(np.exp(-1.0/(q - self.max_joint_angles)), self.max_torque)
        avoid_max[(self.max_joint_angles - q) < 0] = -self.max_torque
        avoid_max[self.no_limits_max] = 0.0

        self.u_avoid = avoid_min + avoid_max

        return self.u_avoid
