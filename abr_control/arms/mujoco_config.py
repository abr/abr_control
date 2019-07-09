import os
import numpy as np

import mujoco_py as mjp
import xml.etree.ElementTree as ET

class MujocoConfig():
    """ Replicates the interface API of base_config, but using the Mujoco
    simulator to perform all the kinematics and dynamics calculations.
    """

    def __init__(self, mjcf_file):
        """
        Imports the xml file to use as the arm model and config

        Parameters
        ----------
        mjcf_file: string
            the name of the arm model to load
            The naming convention is create a folder in this directory
            with your armName and place an armName.xml in it. If any
            stl files are needed, place them in the armName folder in
            a meshes folder. If alternate arm models are available,
            name them as armName_alternate1. The string passed in is
            parsed such that everything up to the first underscore is
            used for the arm directory, and the full string is used to
            load the xml within that folder.

            EX: 'myArm' and 'myArm_with_gripper' will both look in the
            'myArm' directory, however they will load myArm.xml and
            myArm_with_gripper.xml, respectively
        """

        current_dir = os.path.dirname(__file__)
        self.mjcf_file = os.path.join(
            current_dir, mjcf_file.split('_')[0], '%s.xml' % mjcf_file)
        self.model = mjp.load_model_from_path(self.mjcf_file)

        # get access to some of our custom arm parameters from the xml definition
        tree = ET.parse(self.mjcf_file)
        root = tree.getroot()
        for custom in root.findall('custom/numeric'):
            name = custom.get('name')
            if name == 'N_JOINTS':
                self.N_JOINTS = int(custom.get('data'))
            elif name == 'START_ANGLES':
                START_ANGLES = custom.get('data').split(' ')
                self.START_ANGLES = np.array(
                    [float(angle) for angle in START_ANGLES])


    def _connect(self, sim):
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

        self.JOINT_NAMES = self.sim.model.joint_names
        self.N_JOINTS = len(self.JOINT_NAMES)

        # NOTE: Assuming that in the mjcf file every body defined _after_
        # the base_link body is part of the robot
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


    def g(self, q=None):
        """ Returns qfrc_bias variable, which stores the effects of Coriolis,
        centrifugal, and gravitational forces

        Parameters
        ----------
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        """
        return self.sim.data.qfrc_bias


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
        # Emo: You would have to use a finate-difference approximation
        # in the general case, check differences.cpp
        raise NotImplementedError


    def J(self, name, q=None, x=None):
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

        # get the position Jacobian
        self._J3N[:] = self.sim.data.get_body_jacp(name)
        self._J6N[:3] = self._J3N.reshape((3, self.N_JOINTS))
        # get the rotation Jacobian
        self._J3N[:] = self.sim.data.get_body_jacr(name)
        self._J6N[3:] = self._J3N.reshape((3, self.N_JOINTS))

        return self._J6N


    def M(self, q=None):
        """ Returns the inertia matrix in task space

        Parameters
        ----------
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        """
        # stored in mjData.qM, stored in custom sparse format,
        # convert qM to a dense matrix with mj_fullM
        mjp.cymj._mj_fullM(self.model, self._MNN_vector, self.sim.data.qM)
        # TODO: there's a shape like _MNN function or some such, right?
        self._MNN = self._MNN_vector.reshape((self.N_JOINTS, self.N_JOINTS))
        return self._MNN


    def R(self, name, q=None):
        """ Returns the rotation matrix of the specified body
        """
        mjp.cymj._mju_quat2Mat(
            self._R9, self.sim.data.get_body_xquat(name))
        self._R = self._R9.reshape((3, 3))
        return self._R


    def quaternion(self, name, q=None):
        """ Returns the quaternion of the specified body
        """
        return self.sim.data.get_body_xquat(name)


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
        raise NotImplementedError


    def Tx(self, name, q=None, x=None):
        """ Returns the Cartesian coordinates of the specified Mujoco body

        Based off of http://www.mujoco.org/forum/index.php?threads/
        get-orientation-of-body.3543/

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
        return self.sim.data.get_body_xpos(name)


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
        raise NotImplementedError


if __name__ == '__main__':

    from abr_control.interfaces.mujoco import Mujoco

    filename = '/home/tdewolf/src/abr_control/examples/Mujoco/ur5.xml'
    robot_config = MujocoConfig(filename)
    interface = Mujoco(robot_config)
    interface.connect()

    # print(robot_config.M())
    print(robot_config.J('wrist3_link'))
    # print(robot_config.g())
