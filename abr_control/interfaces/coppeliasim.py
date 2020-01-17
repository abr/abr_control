import numpy as np

from abr_control.utils import transformations, download_meshes

from .interface import Interface
from .coppeliasim_files import sim


class CoppeliaSim(Interface):
    """ An interface for CoppeliaSim.

    Implements force control using CoppeliaSim's torque-limiting method.
    Lock-steps the simulation so that it only moves forward one dt
    every time send_forces is called.

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    dt : float, optional (Default: 0.001)
        simulation time step in seconds
    opmode_streaming : boolean, optional (Default: True)
        send set information (torque, position, orientation) without waiting for
        confirmation from CoppeliaSim before continuing, speed increase of about 20%
    """

    def __init__(self, robot_config, dt=0.001, opmode_streaming=True):

        super(CoppeliaSim, self).__init__(robot_config)

        self.q = np.zeros(self.robot_config.N_JOINTS)  # joint angles
        self.dq = np.zeros(self.robot_config.N_JOINTS)  # joint_velocities

        # joint target velocities, as part of the torque limiting control
        # these need to be super high so that the joints are always moving
        # at the maximum allowed torque
        self.joint_target_velocities = np.ones(robot_config.N_JOINTS) * 10000.0

        self.dt = dt  # time step
        self.count = 0  # keep track of how many times send forces is called
        self.misc_handles = {}  # for tracking miscellaneous object handles

        if opmode_streaming:
            self.set_opmode = sim.simx_opmode_streaming
        else:
            self.set_opmode = sim.simx_opmode_blocking

        self.blocking = sim.simx_opmode_blocking

    def connect(self, load_scene=True, force_download=False):
        """ Connect to the current scene open in CoppeliaSim

        Finds the CoppeliaSim references to the joints of the robot.
        Sets the time step for simulation and put into lock-step mode.

        NOTE: the joints and links must follow the naming convention of
        'joint#' and 'link#', starting from 'joint0' and 'link0'

        NOTE: The dt in the CoppeliaSim physics engine must also be specified
        to be less than the dt used here.
        """

        # close any open connections
        sim.simxFinish(-1)
        # Connect to the V-REP continuous server
        self.clientID = sim.simxStart("127.0.0.1", 19997, True, True, 500, 5)

        if self.clientID == -1:
            raise Exception("Failed connecting to CoppeliaSim remote API server")

        if load_scene:
            # if there's a google id, check for files and download if missing
            if self.robot_config.google_id != "None":
                download_meshes.check_and_download(
                    name=self.robot_config.filename,
                    google_id=self.robot_config.google_id,
                    force_download=force_download,
                )
            # load the scene specified in the config
            sim.simxLoadScene(
                self.clientID, self.robot_config.filename, 0, self.blocking
            )

        sim.simxSynchronous(self.clientID, True)

        # get the handles for each joint and set up streaming
        self.joint_handles = [
            sim.simxGetObjectHandle(self.clientID, name, self.blocking)[1]
            for name in self.robot_config.JOINT_NAMES
        ]

        # get handle for target and set up streaming
        _, self.misc_handles["target"] = sim.simxGetObjectHandle(
            self.clientID, "target", self.blocking
        )
        # get handle for hand
        _, self.hand_handle = sim.simxGetObjectHandle(
            self.clientID, "hand", self.blocking
        )

        sim.simxSetFloatingParameter(
            self.clientID,
            sim.sim_floatparam_simulation_time_step,
            self.dt,  # specify a simulation time step
            sim.simx_opmode_oneshot,
        )

        sim.simxSetBooleanParameter(
            self.clientID,
            sim.sim_boolparam_display_enabled,
            True,
            sim.simx_opmode_oneshot,
        )

        # start our simulation in lockstep with our code
        sim.simxStartSimulation(self.clientID, self.blocking)

        print("Connected to CoppeliaSim remote API server")

    def disconnect(self):
        """ Stop and reset the simulation. """

        # stop the simulation
        sim.simxStopSimulation(self.clientID, self.blocking)

        # Before closing the connection to V-REP,
        # make sure that the last command sent out had time to arrive.
        sim.simxGetPingTime(self.clientID)

        # Now close the connection to V-REP:
        sim.simxFinish(self.clientID)
        print("CoppeliaSim connection closed...")

    def get_orientation(self, name):
        """ Returns the orientation of an object in CoppeliaSim

        the Euler angles [radians] are returned in the relative xyz frame.
        http://www.coppeliarobotics.com/helpFiles/en/eulerAngles.htm

        Parameters
        ----------
        name : string
            the name of the object of interest
        """

        if self.misc_handles.get(name, None) is None:
            # if we haven't retrieved the handle previously
            # get the handle and set up streaming
            _, self.misc_handles[name] = sim.simxGetObjectHandle(
                self.clientID, name, self.blocking
            )

        _, orientation = sim.simxGetObjectOrientation(
            self.clientID,
            self.misc_handles[name],
            -1,  # orientation relative to world
            self.blocking,
        )
        return orientation

    def set_orientation(self, name, angles):
        """ Sets the orientation of an object in CoppeliaSim

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

        if self.misc_handles.get(name, None) is None:
            # if we haven't retrieved the handle previously
            # get the handle and set up streaming
            _, self.misc_handles[name] = sim.simxGetObjectHandle(
                self.clientID, name, self.blocking
            )
        _ = sim.simxSetObjectOrientation(
            self.clientID,
            self.misc_handles[name],
            -1,  # orientation relative to world
            angles,
            self.set_opmode,
        )

    def send_forces(self, u):
        """ Apply the specified torque to the robot joints

        Apply the specified torque to the robot joints, move the simulation
        one time step forward, and update the position of the hand object.

        Parameters
        ----------
        u : np.array
            the torques to apply to the robot
        """

        # invert because CoppeliaSim torque directions are opposite of expected
        u *= -1

        for ii, joint_handle in enumerate(self.joint_handles):

            # get the current joint torque
            _, torque = sim.simxGetJointForce(
                self.clientID, joint_handle, self.blocking
            )
            if _ != 0:
                raise Exception("Error retrieving joint torque, " + "return code ", _)

            # if force has changed signs,
            # we need to change the target velocity sign
            if np.sign(torque) * np.sign(u[ii]) <= 0:
                self.joint_target_velocities[ii] = self.joint_target_velocities[ii] * -1
                _ = sim.simxSetJointTargetVelocity(
                    self.clientID,
                    joint_handle,
                    self.joint_target_velocities[ii],
                    self.set_opmode,
                )

            # and now modulate the force
            _ = sim.simxSetJointForce(
                self.clientID,
                joint_handle,
                abs(u[ii]),  # force to apply
                self.set_opmode,
            )

        # Update position of hand object
        hand_xyz = self.robot_config.Tx(name="EE", q=self.q)
        self.set_xyz("hand", hand_xyz)

        # Update orientation of hand object
        angles = transformations.euler_from_matrix(
            self.robot_config.R("EE", q=self.q), axes="rxyz"
        )
        self.set_orientation("hand", angles)

        # move simulation ahead one time step
        sim.simxSynchronousTrigger(self.clientID)
        self.count += self.dt

    def send_target_angles(self, q, joint_handles=None):
        """ Move the robot to the specified configuration.

        Parameters
        ----------
        q : np.array
            configuration to move to [radians]
        joint_handles: list, optional (Default: None)
            ID numbers for the joint, used when trying to get information
            out of the CoppeliaSim remote API
        """

        joint_handles = self.joint_handles if joint_handles is None else joint_handles

        for ii, joint_handle in enumerate(joint_handles):
            # send in angles to move to
            # NOTE: no success / fail message received in oneshot mode
            sim.simxSetJointPosition(
                self.clientID, joint_handle, q[ii], sim.simx_opmode_oneshot
            )

    def get_feedback(self):
        """ Return a dictionary of information needed by the controller.

        Returns the joint angles and joint velocities in [rad] and [rad/sec],
        respectively
        """

        for ii, joint_handle in enumerate(self.joint_handles):
            # get the joint angles
            _, self.q[ii] = sim.simxGetJointPosition(
                self.clientID, joint_handle, self.blocking
            )
            if _ != 0:
                raise Exception("Error retrieving joint angle, " + "return code ", _)

            # get the joint velocity
            _, self.dq[ii] = sim.simxGetObjectFloatParameter(
                self.clientID,
                joint_handle,
                2012,  # ID for joint angular velocity
                self.blocking,
            )
            if _ != 0:
                raise Exception("Error retrieving joint velocity, " + "return code ", _)

        return {"q": self.q, "dq": self.dq}

    def get_xyz(self, name):
        """ Returns the xyz position of the specified object

        name : string
            name of the object you want the xyz position of
        """

        if self.misc_handles.get(name, None) is None:
            # if we haven't retrieved the handle previously
            # get the handle and set up streaming
            _, self.misc_handles[name] = sim.simxGetObjectHandle(
                self.clientID, name, self.blocking
            )

        _, xyz = sim.simxGetObjectPosition(
            self.clientID,
            self.misc_handles[name],
            -1,  # get absolute, not relative position
            self.blocking,
        )
        return xyz

    def set_xyz(self, name, xyz):
        """ Set the position of an object in the environment.

        name : string
            the name of the object
        xyz : np.array
            the [x,y,z] location of the target [meters]
        """

        if self.misc_handles.get(name, None) is None:
            # if we haven't retrieved the handle previously
            # get the handle and set up streaming
            _, self.misc_handles[name] = sim.simxGetObjectHandle(
                self.clientID, name, self.blocking
            )

        sim.simxSetObjectPosition(
            self.clientID,
            self.misc_handles[name],
            -1,  # set absolute, not relative position
            xyz,
            self.set_opmode,
        )
