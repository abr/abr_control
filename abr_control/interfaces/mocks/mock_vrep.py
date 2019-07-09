from abr_control.interfaces.interface import Interface
import numpy as np

# TODO: add ability to load models files so that vrep only has to be open
class MockVREP():
    """ An interface for VREP.

    Implements force control using VREP's torque-limiting method.
    Lock-steps the simulation so that it only moves forward one dt
    every time send_forces is called.

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    dt : float, optional (Default: 0.001)
        simulation time step in seconds
    """

    def __init__(self, robot_config, dt=.001):

        self.robot_config = robot_config
        self.q = np.zeros(self.robot_config.N_JOINTS)  # joint angles
        self.dq = np.zeros(self.robot_config.N_JOINTS)  # joint_velocities
        self.joint_target_velocities = (np.ones(robot_config.N_JOINTS) *
                                        10000.0)
        self.dt = dt  # time step
        self.count = 0  # keep track of how many times send forces is called

    def connect(self):
        """ Connect to the current scene open in VREP

        Finds the VREP references to the joints of the robot.
        Sets the time step for simulation and put into lock-step mode.

        NOTE: the joints and links must follow the naming convention of
        'joint#' and 'link#', starting from 'joint0' and 'link0'

        NOTE: The dt in the VREP physics engine must also be specified
        to be less than the dt used here.
        """

        print('Connected to VREP remote API server')

    def disconnect(self):
        """ Stop and reset the simulation. """
        print('VREP connection closed...')

    def get_orientation(self, name):
        """ Returns the orientation of an object in VREP

        the Euler angles [radians] are returned in the relative xyz frame.
        http://www.coppeliarobotics.com/helpFiles/en/eulerAngles.htm

        Parameters
        ----------
        name : string
            the name of the object of interest
        """

        return np.array([2, 2, 2])

    def set_orientation(self, name, angles):
        """ Sets the orientation of an object in VREP

        Sets the orientation of an object using the provided Euler angles.
        Angles must be in a relative xyz frame.

        Parameters
        ----------
        name : string
            the name of the object of interest
        angles : np.array
            the [alpha, beta, gamma] Euler angles [radians]
            for the object, specified in relative xyz axes.
        """
        assert len(orientation) == 3


    def send_forces(self, u):
        """ Apply the specified torque to the robot joints

        Apply the specified torque to the robot joints, move the simulation
        one time step forward, and update the position of the hand object.

        Parameters
        ----------
        u : np.array
            the torques to apply to the robot
        """
        assert len(u) == self.robot_config.N_JOINTS

    def send_target_angles(self, q, joint_handles=None):
        """ Move the robot to the specified configuration.

        Parameters
        ----------
        q : np.array
            configuration to move to [radians]
        joint_handles: list, optional (Default: None)
            ID numbers for the joint, used when trying to get information
            out of the VREP remote API
        """
        assert len(q) == self.robot_config.N_JOINTS

    def get_feedback(self):
        """ Return a dictionary of information needed by the controller.

        Returns the joint angles and joint velocities in [rad] and [rad/sec],
        respectively
        """

        return {'q': np.ones(self.robot_config.N_JOINTS),
                'dq': np.ones(self.robot_config.N_JOINTS)}

    def get_xyz(self, name):
        """ Returns the xyz position of the specified object

        name : string
            name of the object you want the xyz position of
        """

        return np.array([4, 4, 4])

    def set_xyz(self, name, xyz):
        """ Set the position of an object in the environment.

        name : string
            the name of the object
        xyz : np.array
            the [x,y,z] location of the target [meters]
        """
        assert len(xyz) == 3
