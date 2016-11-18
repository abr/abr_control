import numpy as np

import jaco2_files.jaco2_rs485 as jaco2_rs485
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
        """ All initial setup. """
        self.jaco2.Connect()
        self.jaco2.InitForceMode()

    def disconnect(self):
        """ Any socket closing etc that must be done
        when done with the arm. """
        self.jaco2.Disconnect()

    def apply_u(self, u):
        """ Applies the set of torques u to the arm."""
        self.jaco2.ApplyU(u)

    def get_feedback(self):
        """ Returns a dictionary of relevant feedback to the
        controller. At very least this contains q, dq.
        """
        return self.jaco2.GetFeedback()
