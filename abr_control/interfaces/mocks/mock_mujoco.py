from abr_control.interfaces.interface import Interface
import numpy as np

class MockMujoco(Interface):
    """ An interface for MuJoCo using the mujoco-py package.

    Parameters
    ----------
    robot_config: class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    dt: float, optional (Default: 0.001)
        simulation time step in seconds
    """

    def __init__(self, robot_config, dt=.001):

        self.dt = dt  # time step
        self.count = 0  # keep track of how many times send forces is called

        self.robot_config = robot_config

    def connect(self):
        """
        NOTE: currently it is assumed that all joints are on the robot
        i.e. there are no bodies with freejoints elsewhere in the XML
        """
        print('MuJoCo session created')


    def disconnect(self):
        """ Stop and reset the simulation. """
        # nothing to do to close a MuJoCo session
        print('MuJoCO session closed...')


    def get_mocap_orientation(self, name):
        """ Returns the orientation of an object as the [w x y z]
        quaternion [radians]

        Parameters
        ----------
        name: string
            the name of the object of interest
        """
        return np.array([1, 0, 0, 0])


    def set_mocap_orientation(self, name, quat):
        """ Sets the orientation of an object in the Mujoco environment

        Sets the orientation of an object using the provided Euler angles.
        Angles must be in a relative xyz frame.

        Parameters
        ----------
        name: string
            the name of the object of interest
        quat: np.array
            the [w x y z] quaternion [radians] for the object.
        """
        assert len(quat) == 4


    def send_forces(self, u):
        """ Apply the specified torque to the robot joints

        Apply the specified torque to the robot joints, move the simulation
        one time step forward, and update the position of the hand object.

        Parameters
        ----------
        u: np.array
            the torques to apply to the robot
        """
        assert len(u) == self.robot_config.N_JOINTS

    def send_target_angles(self, q, joint_addrs=None):
        """ Move the robot to the specified configuration.

        Parameters
        ----------
        q: np.array
            configuration to move to [radians]
        joint_addrs: list, optional (Default: None)
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


    def get_mocap_xyz(self, name):
        """ Returns the xyz position of the specified object

        name: string
            name of the object you want the xyz position of
        """
        return np.array([1, 2, 3])


    def set_mocap_xyz(self, name, xyz):
        """ Set the position of a mocap object in the Mujoco environment.

        name: string
            the name of the object
        xyz: np.array
            the [x,y,z] location of the target [meters]
        """
        assert len(xyz) == 3
