from xml.etree import ElementTree

import os
import numpy as np

import mujoco_py as mjp

class MujocoConfig():
    """ A wrapper on the Mujoco simulator to generate all the kinematics and
    dynamics calculations necessary for controllers.
    """

    def __init__(self, xml_file, use_sim_state=False):
        """ Loads the Mujoco model from the specified xml file

        Parameters
        ----------
        xml_file: string
            the name of the arm model to load
            The string passed in is parsed such that everything up to the first
            underscore is used for the arm directory, and the full string is
            used to load the xml within that folder.

            EX: 'myArm' and 'myArm_with_gripper' will both look in the
            'myArm' directory, however they will load myArm.xml and
            myArm_with_gripper.xml, respectively
        use_sim_state: Boolean, optional (Default: False)
            If set true, the q and dq values passed in to the functions are
            ignored, and the current state of the simulator is used to
            calculate all functions. Can speed up simulation by not resetting
            the state on every call.
        """

        current_dir = os.path.dirname(__file__)
        self.xml_file = os.path.join(
            current_dir, xml_file.split('_')[0], '%s.xml' % xml_file)
        self.model = mjp.load_model_from_path(self.xml_file)

        self.use_sim_state = use_sim_state

        # get access to some of our custom arm parameters from the xml definition
        tree = ElementTree.parse(self.xml_file)
        root = tree.getroot()
        for custom in root.findall('custom/numeric'):
            name = custom.get('name')
            if name == 'N_JOINTS':
                self.N_JOINTS = int(custom.get('data'))
            elif name == 'START_ANGLES':
                START_ANGLES = custom.get('data').split(' ')
                self.START_ANGLES = np.array(
                    [float(angle) for angle in START_ANGLES])


    def _connect(self, sim, joint_pos_addrs, joint_vel_addrs):
        """ Called by the interface once the Mujoco simulation is created,
        this connects the config to the simulator so it can access the
        kinematics and dynamics information calculated by Mujoco.

        Parameters
        ----------
        sim: MjSim
            The Mujoco Simulator object created by the Mujoco Interface class
        """
        # get access to the Mujoco simulation
        self.sim = sim
        self.joint_pos_addrs = joint_pos_addrs
        self.joint_vel_addrs = joint_vel_addrs

        self.JOINT_NAMES = self.sim.model.joint_names
        self.N_JOINTS = len(self.JOINT_NAMES)

        # NOTE: We assume that every body defined in the xml _after_
        # 'base_link' is part of the robot
        self.base_link_index = self.sim.model.body_name2id('base_link')
        self.N_LINKS = len(self.sim.model.body_parentid) - self.base_link_index

        # a place to store data returned from Mujoco
        self._g = np.zeros(self.N_JOINTS)
        self._J3N = np.zeros(3 * self.N_JOINTS)
        self._J6N = np.zeros((6, self.N_JOINTS))
        self._MNN_vector = np.zeros(self.N_JOINTS**2)
        self._MNN = np.zeros((self.N_JOINTS, self.N_JOINTS))
        self._R9 = np.zeros(9)
        self._R = np.zeros((3, 3))
        self._x = np.ones(4)


    def _load_state(self, q, dq=None, u=None):
        """ Change the current joint angles

        Parameters
        ----------
        q: np.array
            The set of joint angles to move the arm to
        """
        # save current state
        old_q = np.copy(self.sim.data.qpos[self.joint_pos_addrs])
        old_dq = np.copy(self.sim.data.qvel[self.joint_vel_addrs])
        old_u = np.copy(self.sim.data.ctrl)

        # update positions to specified state
        self.sim.data.qpos[self.joint_pos_addrs] = np.copy(q)
        if dq is not None:
            self.sim.data.qvel[self.joint_vel_addrs] = np.copy(dq)
        if u is not None:
            self.sim.data.ctrl[:] = np.copy(u)

        # move simulation forward to calculate new kinamtic information
        self.sim.forward()

        return old_q, old_dq, old_u


    def g(self, q=None):
        """ Returns qfrc_bias variable, which stores the effects of Coriolis,
        centrifugal, and gravitational forces

        Parameters
        ----------
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        """
        # TODO: For the Coriolis and centrifugal functions, setting the
        # velocity before calculation is important, how best to do this?
        if not self.use_sim_state and q is not None:
            old_q, old_dq, old_u = self._load_state(q)

        g = -1 * np.copy(self.sim.data.qfrc_bias)

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return g


    def dJ(self, name, q=None, dq=None, x=None):
        """ Returns the derivative of the Jacobian wrt to time

        Parameters
        ----------
        name: string
            The name of the Mujoco body to retrieve the Jacobian for
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        dq: float numpy.array, optional (Default: None)
            The joint velocities of the robot. If None the current state is
            retrieved from the Mujoco simulator
        x: float numpy.array, optional (Default: None)
        """
        # TODO if ever required
        # Note from Emo in Mujoco forums:
        # 'You would have to use a finate-difference approximation in the
        # general case, check differences.cpp'
        raise NotImplementedError


    def J(self, name, q=None, x=None, object_type='body'):
        """ Returns the Jacobian for the specified Mujoco body

        Parameters
        ----------
        name: string
            The name of the Mujoco body to retrieve the Jacobian for
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        x: float numpy.array, optional (Default: None)
        """
        if x is not None and not np.allclose(x, 0):
            raise Exception('x offset currently not supported: ', x)

        if not self.use_sim_state and q is not None:
            old_q, old_dq, old_u = self._load_state(q)

        if object_type == 'body':
            jacp = self.sim.data.get_body_jacp
            jacr = self.sim.data.get_body_jacr
        elif object_type == 'geom':
            jacp = self.sim.data.get_geom_jacp
            jacr = self.sim.data.get_geom_jacr
        elif object_type == 'site':
            jacp = self.sim.data.get_site_jacp
            jacr = self.sim.data.get_site_jacr
        else:
            raise Exception('Invalid object type specified: ', object_type)

        # get the position Jacobian hstacked (1 x N_JOINTS*3)
        self._J3N[:] = jacp(name)
        self._J6N[:3] = self._J3N.reshape((3, self.N_JOINTS))
        # get the rotation Jacobian hstacked (1 x N_JOINTS*3)
        self._J3N[:] = jacr(name)
        self._J6N[3:] = self._J3N.reshape((3, self.N_JOINTS))

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return self._J6N


    def M(self, q=None):
        """ Returns the inertia matrix in task space

        Parameters
        ----------
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        """
        if not self.use_sim_state and q is not None:
            old_q, old_dq, old_u = self._load_state(q)

        # stored in mjData.qM, stored in custom sparse format,
        # convert qM to a dense matrix with mj_fullM
        mjp.cymj._mj_fullM(self.model, self._MNN_vector, self.sim.data.qM)
        # TODO: there's a shape like _MNN function or some such, right?
        self._MNN = self._MNN_vector.reshape((self.N_JOINTS, self.N_JOINTS))

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return self._MNN


    def R(self, name, q=None):
        """ Returns the rotation matrix of the specified body
        """
        if not self.use_sim_state and q is not None:
            old_q, old_dq, old_u = self._load_state(q)

        mjp.cymj._mju_quat2Mat(
            self._R9, self.sim.data.get_body_xquat(name))
        self._R = self._R9.reshape((3, 3))

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return self._R


    def quaternion(self, name, q=None):
        """ Returns the quaternion of the specified body
        """
        if not self.use_sim_state and q is not None:
            old_q, old_dq, old_u = self._load_state(q)

        quaternion = np.copy(self.sim.data.get_body_xquat(name))

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return quaternion


    def C(self, q=None):
        """ NOTE: The Coriolis and centrifugal effects (and gravity) are
        already accounted for by Mujoco in the qfrc_bias variable. There's
        no easy way to separate these, so all are returned by the g function.
        To prevent accounting for these effects twice, this function will
        return an error instead of qfrc_bias again.
        """
        raise Exception('Coriolis and centrifugal effects already accounted '
                        + 'for in the term return by the gravity function.')


    def T(self, name, q=None, x=None):
        """ Get the transform matrix of the specified body.

        Parameters
        ----------
        name: string
            The name of the Mujoco body to retrieve the Jacobian for
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        x: float numpy.array, optional (Default: None)
        """
        # TODO if ever required
        raise NotImplementedError


    def Tx(self, name, q=None, x=None, object_type='body'):
        """ Returns the Cartesian coordinates of the specified Mujoco body

        Parameters
        ----------
        name: string
            The name of the Mujoco body to retrieve the Jacobian for
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        x: float numpy.array, optional (Default: None)
        """
        if x is not None and not np.allclose(x, 0):
            raise Exception('x offset currently not supported: ', x)

        if not self.use_sim_state and q is not None:
            old_q, old_dq, old_u = self._load_state(q)

        if object_type == 'body':
            Tx = np.copy(self.sim.data.get_body_xpos(name))
        elif object_type == 'geom':
            Tx = np.copy(self.sim.data.get_geom_xpos(name))
        elif object_type == 'joint':
            Tx = np.copy(self.sim.data.get_joint_xanchor(name))
        elif object_type == 'site':
            Tx = np.copy(self.sim.data.get_site_xpos(name))
        elif object_type == 'camera':
            Tx = np.copy(self.sim.data.get_cam_xpos(name))
        elif object_type == 'light':
            Tx = np.copy(self.sim.data.get_light_xpos(name))
        elif object_type == 'mocap':
            Tx = np.copy(self.sim.data.get_mocap_pos(name))
        else:
            raise Exception('Invalid object type specified: ', object_type)

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return Tx


    def T_inv(self, name, q=None, x=None):
        """  Get the inverse transform matrix of the specified body.

        Parameters
        ----------
        name: string
            The name of the Mujoco body to retrieve the Jacobian for
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        x: float numpy.array, optional (Default: None)
        """
        # TODO if ever required
        raise NotImplementedError
