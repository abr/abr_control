import numpy as np
from abr_control.utils import download_meshes, transformations
from isaacsim import SimulationApp
from .interface import Interface


class IsaacSim(Interface):
    """An interface for IsaacSim.

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    dt : float, optional (Default: 0.001)
        simulation time step in seconds

    """
    def __init__(self, robot_config, dt=0.001):

        super().__init__(robot_config)

        self.dt = dt  # time step
        self.count = 0  # keep track of how many times send forces is called

        '''
        self.q = np.zeros(self.robot_config.N_JOINTS)  # joint angles
        self.dq = np.zeros(self.robot_config.N_JOINTS)  # joint_velocities

        # joint target velocities, as part of the torque limiting control
        # these need to be super high so that the joints are always moving
        # at the maximum allowed torque
        self.joint_target_velocities = np.ones(robot_config.N_JOINTS) * 10000.0
        '''
        



    def connect(self):
        """All initial setup."""
        self.simulation_app = SimulationApp({"headless": False}) 
        print("Started Isaacsim as stand alone app...")

    def disconnect(self):
        """Any socket closing etc that must be done to properly shut down"""
        self.simulation_app.close() # close Isaac Sim
        print("IsaacSim connection closed...")

    def send_forces(self, u):
        """Applies the set of torques u to the arm. If interfacing to
        a simulation, also moves dynamics forward one time step.

        u : np.array
            An array of joint torques [Nm]
        """

        raise NotImplementedError

    def send_target_angles(self, q):
        """Moves the arm to the specified joint angles

        q : numpy.array
            the target joint angles [radians]
        """

        raise NotImplementedError

    def get_feedback(self):
        """Returns a dictionary of the relevant feedback

        Returns a dictionary of relevant feedback to the
        controller. At very least this contains q, dq.
        """

        raise NotImplementedError
