import numpy as np

import mujoco_py as mjp

from mujoco_py.generated import const

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
    visualize: boolean, optional (Default: True)
        turns visualization on or off
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


    def connect(self, camera_id=-1):
        """
        NOTE: currently it is assumed that all joints are on the robot
        i.e. there are no bodies with freejoints elsewhere in the XML

        camera_id: int, optional (Default: -1)
            the id of the camera to use for the visualization
        """
        self.sim = mjp.MjSim(self.robot_config.model)
        self.sim.forward()  # run forward to fill in sim.data
        model = self.sim.model

        # get the kinematic tree for the arm
        joint_ids = []
        joint_names = []
        body_id = model.body_name2id('EE')
        # start with the end-effector (EE) and work back to the world body
        while model.body_parentid[body_id] != 0:
            jntadrs_start = model.body_jntadr[body_id]
            for ii in range(model.body_jntnum[body_id]):
                joint_ids.append(jntadrs_start + ii)
                joint_names.append(model.joint_id2name(joint_ids[-1]))
            body_id = model.body_parentid[body_id]
        # flip the list so it starts with the base of the arm / first joint
        joint_names = np.array(joint_names[::-1])
        joint_ids = np.array(joint_ids[::-1])

        self.joint_pos_addrs = [model.get_joint_qpos_addr(name)
                                for name in joint_names]

        self.joint_vel_addrs = [model.get_joint_qvel_addr(name)
                                for name in joint_names]

        # Need to also get the joint rows of the Jacobian, inertia matrix, and
        # gravity vector. This is trickier because if there's a quaternion in
        # the joint (e.g. a free joint or a ball joint) then the joint position
        # address will be different than the joint Jacobian row. This is because
        # the quaternion joint will have a 4D position and a 3D derivative. So
        # we go through all the joints, and find out what type they are, then
        # calculate the Jacobian position based on their order and type.
        index = 0
        self.joint_dyn_addrs = []
        for ii, joint_type in enumerate(model.jnt_type):
            if ii in joint_ids:
                self.joint_dyn_addrs.append(index)
            if joint_type == 0:  # free joint
                index += 6  # derivative has 6 dimensions
            elif joint_type == 1:  # ball joint
                index += 3  # derivative has 3 dimensions
            else:  # slide or hinge joint
                index += 1  # derivative has 1 dimensions


        # give the robot config access to the sim for wrapping the
        # forward kinematics / dynamics functions
        self.robot_config._connect(self.sim,
                                   self.joint_pos_addrs,
                                   self.joint_vel_addrs,
                                   self.joint_dyn_addrs)

        # create the visualizer
        if self.visualize:
            self.viewer = mjp.MjViewer(self.sim)
            # if specified, set the camera
            if camera_id > -1:
                self.viewer.cam.type = const.CAMERA_FIXED
                self.viewer.cam.fixedcamid = camera_id

        print('MuJoCo session created')


    def disconnect(self):
        """ Stop and reset the simulation. """
        # nothing to do to close a MuJoCo session
        print('MuJoCO session closed...')


    def get_orientation(self, name, object_type='body'):
        """ Returns the orientation of an object as the [w x y z] quaternion [radians]

        Parameters
        ----------
        name: string
            the name of the object of interest
        """
        if object_type == 'mocap':  # commonly queried to find target
            quat = self.sim.data.get_mocap_quat(name)
        elif object_type == 'body':
            body_id = self.sim.model.body_name2id(name)
            quat = self.sim.model._body_quat[body_id]
        elif object_type == 'geom':
            geom_id = self.sim.model.geom_name2id(name)
            quat = self.sim.model.geom_quat[geom_id]
        elif object_type == 'site':
            site_id = self.sim.model.site_name2id(name)
            quat = self.sim.model._site_quat[site_id]
        else:
            raise Exception('get_orientation for %s object type not supported'
                            % object_type)
        return quat


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


    def send_forces(self, u, update_display=True):
        """ Apply the specified torque to the robot joints

        Apply the specified torque to the robot joints, move the simulation
        one time step forward, and update the position of the hand object.

        Parameters
        ----------
        u: np.array
            the torques to apply to the robot [Nm]
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

        if self.visualize and update_display:
            self.viewer.render()
        self.count += self.dt


    def send_target_angles(self, q):
        """ Move the robot to the specified configuration.

        Parameters
        ----------
        q: np.array
            configuration to move to [radians]
        """

        self.sim.data.qpos[self.joint_pos_addrs] = np.copy(q)
        self.sim.forward()


    def set_joint_state(self, q, dq):
        """ Move the robot to the specified configuration.

        Parameters
        ----------
        q: np.array
            configuration to move to [rad]
        dq: np.array
            joint velocities [rad/s]
        """

        self.sim.data.qpos[self.joint_pos_addrs] = np.copy(q)
        self.sim.data.qvel[self.joint_vel_addrs] = np.copy(dq)
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
