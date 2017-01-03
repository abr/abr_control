import numpy as np

from .jaco2_files import jaco2_rs485
from abr_control.interfaces import interface


class interface(interface.interface):
    """ Base class for interfaces.
    The purpose of interfaces is to abstract away the API
    and overhead of connection / disconnection etc for each
    of the different systems that can be controlled.
    """

    def __init__(self, robot_config):
        super(interface, self).__init__(robot_config)
        self.jaco2 = jaco2_rs485.pyJaco2()

    def connect(self):
        # TODO: send arm to home position before calling initforcemode
        """ All initial setup. """
        self.jaco2.Connect()

    def disconnect(self):
        """ Any socket closing etc that must be done
        when done with the arm. """
        self.jaco2.Disconnect()

    def apply_u(self, u):
        """ Applies the set of torques u to the arm.

        NOTE: if a torque is not applied every 200ms then
        the arm reverts back to position control and the
        InitForceMode function must be called again.
        """
        self.jaco2.ApplyU(u)

    def get_feedback(self):
        """ Returns a dictionary of relevant feedback to the
        controller. At very least this contains q, dq.
        """
        return self.jaco2.GetFeedback()

    def apply_q(self, q):
        """ Moves the arm to the specified joint angles using
        the on-board PD controller.

        """
        self.jaco2.ApplyQ(q)

    def init_force_mode(self, expected_torque=None):
        """ Changes the arm to torque control mode, where forces
        can be sent in specifying what torque to apply at each joint.

        expected_torque np.array: the torque expected at each motor
                                  in the current position
        """

        if expected_torque is None:
            # 2lb weight + hand
            expected_torque = np.array(
                [1.0, -0.8, 0.9, 0.0, 0.0, 0.0],
                dtype="float32")
            # 3lb weight + hand
            # expected_torque = np.array(
            #     [0.9, -2.4, 1.9, 0.0, 0.0, 0.0],
            #     dtype="float32"))

        self.jaco2.InitForceMode(expected_torque)

    def init_position_mode(self):
        """ Changes the arm into position control mode, where
        target joint angles can be provided, and the on-board PD
        controller will move the arm.
        """
        self.jaco2.InitPositionMode()

    def get_pos(self):
        """ Returns the last set of joint angles and velocities
        read in from the arm as a dictionary, with keys 'q' and 'dq'.
        """
        return self.jaco2.GetFeedback()
