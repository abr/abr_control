import numpy as np
from .vrep_files import vrep

import abr_control
from abr_control.utils import transformations
from .interface import Interface


# TODO: add ability to load models files so that vrep only has to be open
class VREP(Interface):
    """ An interface for VREP.

    Implements force control using VREP's force-limiting method.
    Lock-steps the simulation so that it only moves forward one dt
    every time send_forces is called.
    Expects that there is a 'hand' object in the VREP environment

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    dt : float, optional (Default: 0.001)
        simulation timestep in seconds
    """

    def __init__(self, robot_config, dt=.001):

        super(VREP, self).__init__(robot_config)

        self.q = np.zeros(self.robot_config.N_JOINTS)  # joint angles
        self.dq = np.zeros(self.robot_config.N_JOINTS)  # joint_velocities

        # joint target velocities, as part of the torque limiting control
        # these need to be super high so that the joints are always moving
        # at the maximum allowed torque
        self.joint_target_velocities = (np.ones(robot_config.N_JOINTS) *
                                        10000.0)

        self.dt = dt  # time step
        self.count = 0  # keep track of how many times send forces has been called
        self.misc_handles = {}  # for tracking miscellaneous object handles

    def connect(self):
        """ Connect to the current scene open in VREP

        find the VREP references to the joints of the robot,
        specify the time step for simulation and put into lock-step mode.

        NOTE: the joints and links must follow the naming convention of
        'joint#' and 'link#', starting from 'joint0' and 'link0'

        NOTE: The dt in the VREP physics engine must also be specified
        to be less than the dt used here.
        """

        # close any open connections
        vrep.simxFinish(-1)
        # Connect to the V-REP continuous server
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)

        if self.clientID == -1:
            raise Exception('Failed connecting to remote API server')

        vrep.simxSynchronous(self.clientID, True)

        # get the handles for each joint and set up streaming
        self.joint_handles = [vrep.simxGetObjectHandle(self.clientID,
                              name,
                              vrep.simx_opmode_blocking)[1] for name in
                              self.robot_config.JOINT_NAMES]

        # get handle for target and set up streaming
        _, self.misc_handles['target'] = \
            vrep.simxGetObjectHandle(self.clientID,
                                     'target',
                                     vrep.simx_opmode_blocking)
        # get handle for hand
        _, self.hand_handle = \
            vrep.simxGetObjectHandle(self.clientID,
                                     'hand',
                                     vrep.simx_opmode_blocking)

        vrep.simxSetFloatingParameter(
            self.clientID,
            vrep.sim_floatparam_simulation_time_step,
            self.dt,  # specify a simulation time step
            vrep.simx_opmode_oneshot)

        vrep.simxSetBooleanParameter(
            self.clientID,
            vrep.sim_boolparam_display_enabled,
            True,
            vrep.simx_opmode_oneshot)

        # start our simulation in lockstep with our code
        vrep.simxStartSimulation(self.clientID,
                                 vrep.simx_opmode_blocking)

        print('Connected to VREP remote API server')

    def disconnect(self):
        """ Stop and reset the simulation. """

        # stop the simulation
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)

        # Before closing the connection to V-REP,
        # make sure that the last command sent out had time to arrive.
        vrep.simxGetPingTime(self.clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(self.clientID)
        print('VREP connection closed...')

    def get_orientation(self, name):
        """ Returns the orientation of an object in VREP

        the Euler angles [radians] are returned in the relative xyz frame.

        Parameters
        ----------
        name : string
            the name of the object of interest
        """

        if self.misc_handles.get(name, None) is None:
            # if we haven't retrieved the handle previously
            # get the handle and set up streaming
            _, self.misc_handles[name] = \
                vrep.simxGetObjectHandle(self.clientID,
                                         name,
                                         vrep.simx_opmode_blocking)
        _, orientation = \
            vrep.simxGetObjectOrientation(
                self.clientID,
                self.misc_handles[name],
                -1,  # orientation relative to world
                vrep.simx_opmode_blocking)
        return orientation

    def set_orientation(self, name, angles):
        """ Sets the orientation of an object in VREP

        Sets the orientation of an object using the provided Euler angles .

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
            _, self.misc_handles[name] = \
                vrep.simxGetObjectHandle(self.clientID,
                                         name,
                                         vrep.simx_opmode_blocking)
        vrep.simxSetObjectOrientation(
            self.clientID,
            self.misc_handles[name],
            -1,  # orientation relative to world
            angles,
            vrep.simx_opmode_blocking)

    def send_forces(self, u):
        """ Apply the specified torque to the robot joints

        Apply the specified torque to the robot joints,
        move the simulation one time step forward, and update
        the position of the hand object.

        Parameters
        ----------
        u : np.array
            an array of the torques to apply to the robot
        """

        # invert because torque directions are opposite of expected
        u *= -1

        for ii, joint_handle in enumerate(self.joint_handles):

            # get the current joint torque
            _, torque = vrep.simxGetJointForce(
                self.clientID,
                joint_handle,
                vrep.simx_opmode_blocking)
            if _ != 0:
                raise Exception('Error retrieving joint torque.')

            # if force has changed signs,
            # we need to change the target velocity sign
            if np.sign(torque) * np.sign(u[ii]) <= 0:
                self.joint_target_velocities[ii] = \
                    self.joint_target_velocities[ii] * -1
                _ = vrep.simxSetJointTargetVelocity(
                    self.clientID,
                    joint_handle,
                    self.joint_target_velocities[ii],
                    vrep.simx_opmode_blocking)
                if _ != 0:
                    raise Exception('Error setting joint target velocity.')

            # and now modulate the force
            _ = vrep.simxSetJointForce(self.clientID,
                                       joint_handle,
                                       abs(u[ii]),  # force to apply
                                       vrep.simx_opmode_blocking)
            if _ != 0:
                raise Exception('Error setting max joint force.')

        # Update position of hand object
        hand_xyz = self.robot_config.Tx(name='EE', q=self.q)
        self.set_xyz('hand', hand_xyz)

        # Update orientation of hand object
        quaternion = self.robot_config.orientation('EE', q=self.q)
        angles = transformations.euler_from_quaternion(
            quaternion, axes='rxyz')
        self.set_orientation('hand', angles)

        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self.clientID)
        self.count += self.dt

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

        joint_handles = (self.joint_handles if joint_handles is None
                         else joint_handles)

        for ii, joint_handle in enumerate(joint_handles):
            # send in angles to move to
            # NOTE: no success / fail message received in oneshot mode
            vrep.simxSetJointPosition(
                self.clientID,
                joint_handle,
                q[ii],
                vrep.simx_opmode_oneshot)

    def get_feedback(self):
        """ Return a dictionary of information needed by the controller.

        Returns the joint angles and joint velocities in [rad] and [rad/sec]
        respectively
        """

        for ii, joint_handle in enumerate(self.joint_handles):

            # get the joint angles
            _, self.q[ii] = vrep.simxGetJointPosition(
                self.clientID,
                joint_handle,
                vrep.simx_opmode_blocking)
            if _ != 0:
                raise Exception('Error retrieving joint angle.')

            # get the joint velocity
            _, self.dq[ii] = vrep.simxGetObjectFloatParameter(
                self.clientID,
                joint_handle,
                2012,  # ID for joint angular velocity
                vrep.simx_opmode_blocking)
            if _ != 0:
                raise Exception('Error retrieving joint velocity.')

        return {'q': self.q,
                'dq': self.dq}

    def get_xyz(self, name):
        """ Returns the xyz position of the specified object

        name : string
            name of the object you want the xyz position of
        """

        if self.misc_handles.get(name, None) is None:
            # if we haven't retrieved the handle previously
            # get the handle and set up streaming
            _, self.misc_handles[name] = \
                vrep.simxGetObjectHandle(self.clientID,
                                         name,
                                         vrep.simx_opmode_blocking)

        _, xyz = vrep.simxGetObjectPosition(
            self.clientID,
            self.misc_handles[name],
            -1,  # get absolute, not relative position
            vrep.simx_opmode_blocking)
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
            _, self.misc_handles[name] = \
                vrep.simxGetObjectHandle(self.clientID,
                                         name,
                                         vrep.simx_opmode_blocking)

        vrep.simxSetObjectPosition(
            self.clientID,
            self.misc_handles[name],
            -1,  # set absolute, not relative position
            xyz,
            vrep.simx_opmode_blocking)
