import numpy as np

import mujoco_py as mjp

from .interface import Interface


class Mujoco(Interface):
    """ An interface for MuJoCo using the mujoco-py package.

    Parameters
    ----------
    robot_config: class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    dt: float, optional (Default: 0.001)
        simulation time step in seconds
    """

    def __init__(self, robot_config, dt=.001, visualize=True):

        super(Mujoco, self).__init__(robot_config)

        self.dt = dt  # time step
        self.count = 0  # keep track of how many times send forces is called

        self.robot_config = robot_config
        # set the time step for simulation
        self.robot_config.model.opt.timestep = self.dt

        # turns the visualization on or off
        self.visualize = visualize


    def connect(self):
        """
        NOTE: currently it is assumed that all joints are on the robot
        i.e. there are no bodies with freejoints elsewhere in the XML
        """
        self.sim = mjp.MjSim(self.robot_config.model)
        self.sim.forward()  # run forward to fill in sim.data

        self.joint_pos_addrs = [self.sim.model.get_joint_qpos_addr(name)
                                for name in self.sim.model.joint_names]

        self.joint_vel_addrs = [self.sim.model.get_joint_qvel_addr(name)
                                for name in self.sim.model.joint_names]

        # give the robot config access to the sim for wrapping the
        # forward kinematics / dynamics functions
        self.robot_config._connect(self.sim,
                                   self.joint_pos_addrs,
                                   self.joint_vel_addrs)

        # create the visualizer
        if self.visualize:
            self.viewer = mjp.MjViewer(self.sim)

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
        return self.sim.data.get_mocap_quat(name)


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
        self.sim.data.set_mocap_quat(name, quat)


    def send_forces(self, u):
        """ Apply the specified torque to the robot joints

        Apply the specified torque to the robot joints, move the simulation
        one time step forward, and update the position of the hand object.

        Parameters
        ----------
        u: np.array
            the torques to apply to the robot
        """

        # NOTE: the qpos_addr's are unrelated to the order of the motors
        # NOTE: assuming that the robot arm motors are the first len(u) values
        self.sim.data.ctrl[:] = u[:]

        # move simulation ahead one time step
        self.sim.step()

        # Update position of hand object
        # NOTE: contact exclude tags must be included in the XML file
        # for the hand and target with all robot bodies to prevent collisions
        hand_xyz = self.robot_config.Tx(name='EE')
        self.set_mocap_xyz('hand', hand_xyz)

        # Update orientation of hand object
        hand_quat = self.robot_config.quaternion(name='EE')
        self.set_mocap_orientation('hand', hand_quat)

        if self.visualize:
            self.viewer.render()
        self.count += self.dt


    def send_target_angles(self, q):
        """ Move the robot to the specified configuration.

        Parameters
        ----------
        q: np.array
            configuration to move to [radians]
        joint_addrs: list, optional (Default: None)
            ID numbers for the joint, used when trying to get information
            out of the VREP remote API
        """

        self.sim.data.qpos[self.joint_pos_addrs] = np.copy(q)
        self.sim.forward()


    def set_joint_state(self, q, dq):
        """ Move the robot to the specified configuration.

        Parameters
        ----------
        q: np.array
            configuration to move to [radians]
        """

        self.sim.data.qpos[self.joint_pos_addrs] = np.copy(q)
        self.sim.data.qvel[self.joint_vel_addrs] = np.copy(dq)
        # mjp.cymj._mj_step1(self.robot_config.model, self.sim.data)
        self.sim.forward()


    def get_feedback(self):
        """ Return a dictionary of information needed by the controller.

        Returns the joint angles and joint velocities in [rad] and [rad/sec],
        respectively
        """

        self.q = np.copy(self.sim.data.qpos[self.joint_pos_addrs])
        self.dq = np.copy(self.sim.data.qvel[self.joint_vel_addrs])

        return {'q': self.q,
                'dq': self.dq}


    def get_mocap_xyz(self, name):
        """ Returns the xyz position of the specified object

        name: string
            name of the object you want the xyz position of
        """
        return self.sim.data.get_mocap_pos(name)


    def set_mocap_xyz(self, name, xyz):
        """ Set the position of a mocap object in the Mujoco environment.

        name: string
            the name of the object
        xyz: np.array
            the [x,y,z] location of the target [meters]
        """
        self.sim.data.set_mocap_pos(name, xyz)
