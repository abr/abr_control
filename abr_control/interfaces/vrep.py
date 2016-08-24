import numpy as np
import vrep.vrep

import interface


class interface(interface.interface):
    """ An interface for VREP.
    Implements force control using VREP's force-limiting method.
    Lock-steps the simulation so that it only moves forward one dt
    every time apply_u is called.
    Expects that there are there is a 'hand' object in the VREP
    environment,
    """

    def __init__(self, robot_config, dt=.001):
        super(interface, self).__init__(robot_config)

        self.q = np.zeros(self.robot_config.num_joints)  # joint angles
        self.dq = np.zeros(self.robot_config.num_joints)  # joint_velocities

        # joint target velocities, as part of the torque limiting control
        # these need to be super high so that the joints are always moving
        # at the maximum allowed torque
        self.joint_target_velocities = (np.ones(robot_config.num_joints) *
                                        10000.0)

        self.dt = dt  # time step
        self.count = 0  # keep track of how many times apply_u has been called

        # for plotting post run
        self.track_q = []
        self.track_dq = []
        self.track_hand = []

    def connect(self):
        """ Connect to the current scene open in VREP,
        find the VREP references to the joints of the robot,
        specify the time step for simulation and put into lock-step mode.

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
                              self.robot_config.joint_names]

        # get handle for target and set up streaming
        _, self.target_handle = \
            vrep.simxGetObjectHandle(self.clientID,
                                     'target',
                                     vrep.simx_opmode_blocking)
        # get handle for hand
        _, self.hand_handle = \
            vrep.simxGetObjectHandle(self.clientID,
                                     'hand',
                                     vrep.simx_opmode_blocking)

        dt = .001
        vrep.simxSetFloatingParameter(
            self.clientID,
            vrep.sim_floatparam_simulation_time_step,
            dt,  # specify a simulation time step
            vrep.simx_opmode_oneshot)

        # start our simulation in lockstep with our code
        vrep.simxStartSimulation(self.clientID,
                                 vrep.simx_opmode_blocking)

        print('Connected to remote API server')

    def disconnect(self):
        """ Stop and reset the simulation. """
        # stop the simulation
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)

        # Before closing the connection to V-REP,
        # make sure that the last command sent out had time to arrive.
        vrep.simxGetPingTime(self.clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(self.clientID)
        print('connection closed...')

    def apply_u(self, u):
        """ Apply the specified torque to the robot joints,
        move the simulation one time step forward, and update
        the position of the hand object.

        u np.array: an array of the torques to apply to the robot
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
                raise Exception()

            # if force has changed signs,
            # we need to change the target velocity sign
            if np.sign(torque) * np.sign(u[ii]) <= 0:
                self.joint_target_velocities[ii] = \
                    self.joint_target_velocities[ii] * -1
                vrep.simxSetJointTargetVelocity(
                    self.clientID,
                    joint_handle,
                    self.joint_target_velocities[ii],
                    vrep.simx_opmode_blocking)
            if _ != 0:
                raise Exception()

            # and now modulate the force
            vrep.simxSetJointForce(self.clientID,
                                   joint_handle,
                                   abs(u[ii]),  # force to apply
                                   vrep.simx_opmode_blocking)
            if _ != 0:
                raise Exception()

        hand_xyz = self.robot_config.T(self.q)
        # Update position of hand sphere
        vrep.simxSetObjectPosition(
            self.clientID,
            self.hand_handle,
            -1,  # set absolute, not relative position
            hand_xyz,
            vrep.simx_opmode_blocking)

        self.track_hand.append(np.copy(hand_xyz))

        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(self.clientID)
        self.count += self.dt

    def get_feedback(self):
        """ Return a dictionary of information needed by the controller. """
        for ii, joint_handle in enumerate(self.joint_handles):

            # get the joint angles
            _, self.q[ii] = vrep.simxGetJointPosition(
                self.clientID,
                joint_handle,
                vrep.simx_opmode_blocking)
            if _ != 0:
                raise Exception()

            # get the joint velocity
            _, self.dq[ii] = vrep.simxGetObjectFloatParameter(
                self.clientID,
                joint_handle,
                2012,  # ID for joint angular velocity
                vrep.simx_opmode_blocking)
            if _ != 0:
                raise Exception()

        self.track_q.append(np.copy(self.q))
        self.track_dq.append(np.copy(self.dq))

        return {'q': self.q,
                'dq': self.dq}

    def set_target(self, xyz):
        """ Set the position of the target object.

        xyz np.array: the [x,y,z] location of the target (in meters)
        """
        vrep.simxSetObjectPosition(
            self.clientID,
            self.target_handle,
            -1,  # set absolute, not relative position
            xyz,
            vrep.simx_opmode_blocking)
