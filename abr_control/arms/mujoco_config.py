import os
from xml.etree import ElementTree

import mujoco_py as mjp
import numpy as np

from abr_control.utils import download_meshes


class MujocoConfig:
    """A wrapper on the Mujoco simulator to generate all the kinematics and
    dynamics calculations necessary for controllers.
    """

    def __init__(self, xml_file, folder=None, use_sim_state=True, force_download=False):
        """Loads the Mujoco model from the specified xml file

        Parameters
        ----------
        xml_file: string
            the name of the arm model to load. If folder remains as None,
            the string passed in is parsed such that everything up to the first
            underscore is used for the arm directory, and the full string is
            used to load the xml within that folder.

            EX: 'myArm' and 'myArm_with_gripper' will both look in the
            'myArm' directory, however they will load myArm.xml and
            myArm_with_gripper.xml, respectively

            If a folder is passed in, then folder/xml_file is used
        folder: string, Optional (Default: None)
            specifies what folder to find the xml_file, if None specified will
            checking in abr_control/arms/xml_file (see above for xml_file)
        use_sim_state: Boolean, optional (Default: True)
            If set False, the state information is provided by the user, which
            is then used to calculate the corresponding dynamics values.
            The state is then set back to the sim state prior to the user
            provided state.
            If set true, any q and dq values passed in to the functions are
            ignored, and the current state of the simulator is used to
            calculate all functions. This can speed up the simulation, because
            the step of resetting the state on every call is omitted.
        force_download: boolean, Optional (Default: False)
            True to force downloading the mesh and texture files, useful when new files
            are added that may be missing.
            False: if the meshes folder is missing it will ask the user whether they
            want to download them
        """

        if folder is None:
            arm_dir = xml_file.split("_")[0]
            current_dir = os.path.dirname(__file__)
            self.xml_file = os.path.join(current_dir, arm_dir, f"{xml_file}.xml")
            self.xml_dir = f"{current_dir}/{arm_dir}"
        else:
            self.xml_dir = f"{folder}"
            self.xml_file = os.path.join(self.xml_dir, xml_file)

        self.N_GRIPPEPR_JOINTS = 0

        # get access to some of our custom arm parameters from the xml definition
        tree = ElementTree.parse(self.xml_file)
        root = tree.getroot()
        for custom in root.findall("custom/numeric"):
            name = custom.get("name")
            if name == "START_ANGLES":
                START_ANGLES = custom.get("data").split(" ")
                self.START_ANGLES = np.array([float(angle) for angle in START_ANGLES])
            elif name == "N_GRIPPER_JOINTS":
                self.N_GRIPPER_JOINTS = int(custom.get("data"))

        # check for google_id specifying download location of robot mesh files
        self.google_id = None
        for custom in root.findall("custom/text"):
            name = custom.get("name")
            if name == "google_id":
                self.google_id = custom.get("data")

        # check if the user has downloaded the required mesh files
        # if not prompt them to do so
        if self.google_id is not None:
            # get list of expected files to check if all have been downloaded
            files = []
            for asset in root.findall("asset/mesh"):
                files.append(asset.get("file"))

            for asset in root.findall("asset/texture"):
                # assuming that texture are placed in the meshes folder
                files.append(asset.get("file").split("/")[1])

            # check if our mesh folder exists, then check we have all the files
            download_meshes.check_and_download(
                name=self.xml_dir + "/meshes",
                google_id=self.google_id,
                force_download=force_download,
                files=files,
            )

        self.model = mjp.load_model_from_path(self.xml_file)
        self.use_sim_state = use_sim_state

    def _connect(self, sim, joint_pos_addrs, joint_vel_addrs, joint_dyn_addrs):
        """Called by the interface once the Mujoco simulation is created,
        this connects the config to the simulator so it can access the
        kinematics and dynamics information calculated by Mujoco.

        Parameters
        ----------
        sim: MjSim
            The Mujoco Simulator object created by the Mujoco Interface class
        joint_pos_addrs: np.array of ints
            The index of the robot joints in the Mujoco simulation data joint
            position array
        joint_vel_addrs: np.array of ints
            The index of the robot joints in the Mujoco simulation data joint
            velocity array
        joint_dyn_addrs: np.array of ints
            The index of the robot joints in the Mujoco simulation data joint
            Jacobian, inertia matrix, and gravity vector
        """
        # get access to the Mujoco simulation
        self.sim = sim
        self.joint_pos_addrs = np.copy(joint_pos_addrs)
        self.joint_vel_addrs = np.copy(joint_vel_addrs)
        self.joint_dyn_addrs = np.copy(joint_dyn_addrs)

        # number of controllable joints in the robot arm
        self.N_JOINTS = len(self.joint_dyn_addrs)
        # number of joints in the Mujoco simulation
        N_ALL_JOINTS = self.sim.model.nv

        # need to calculate the joint_dyn_addrs indices in flat vectors returned
        # for the Jacobian
        self.jac_indices = np.hstack(
            # 6 because position and rotation Jacobians are 3 x N_JOINTS
            [self.joint_dyn_addrs + (ii * N_ALL_JOINTS) for ii in range(3)]
        )

        # for the inertia matrix
        self.M_indices = [
            ii * N_ALL_JOINTS + jj
            for jj in self.joint_dyn_addrs
            for ii in self.joint_dyn_addrs
        ]

        # a place to store data returned from Mujoco
        self._g = np.zeros(self.N_JOINTS)
        self._J3NP = np.zeros(3 * N_ALL_JOINTS)
        self._J3NR = np.zeros(3 * N_ALL_JOINTS)
        self._J6N = np.zeros((6, self.N_JOINTS))
        self._MNN_vector = np.zeros(N_ALL_JOINTS ** 2)
        self._MNN = np.zeros(self.N_JOINTS ** 2)
        self._R9 = np.zeros(9)
        self._R = np.zeros((3, 3))
        self._x = np.ones(4)
        self.N_ALL_JOINTS = N_ALL_JOINTS

    def _load_state(self, q, dq=None, u=None):
        """Change the current joint angles

        Parameters
        ----------
        q: np.array
            The set of joint angles to move the arm to [rad]
        dq: np.array
            The set of joint velocities to move the arm to [rad/sec]
        u: np.array
            The set of joint forces to apply to the arm joints [Nm]
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

        # move simulation forward to calculate new kinematic information
        self.sim.forward()

        return old_q, old_dq, old_u

    def g(self, q=None):
        """Returns qfrc_bias variable, which stores the effects of Coriolis,
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

        g = -1 * self.sim.data.qfrc_bias[self.joint_dyn_addrs]

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return g

    def dJ(self, name, q=None, dq=None, x=None):
        """Returns the derivative of the Jacobian wrt to time

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

    def J(self, name, q=None, x=None, object_type="body"):
        """Returns the Jacobian for the specified Mujoco object

        Parameters
        ----------
        name: string
            The name of the Mujoco body to retrieve the Jacobian for
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        x: float numpy.array, optional (Default: None)
        object_type: string, the Mujoco object type, optional (Default: body)
            options: body, geom, site
        """
        if x is not None and not np.allclose(x, 0):
            raise Exception("x offset currently not supported, set to None")

        if not self.use_sim_state and q is not None:
            old_q, old_dq, old_u = self._load_state(q)

        if object_type == "body":
            # TODO: test if using this function is faster than the old way
            # NOTE: for bodies, the Jacobian for the COM is returned
            mjp.cymj._mj_jacBodyCom(
                self.model,
                self.sim.data,
                self._J3NP,
                self._J3NR,
                self.model.body_name2id(name),
            )
        else:
            if object_type == "geom":
                jacp = self.sim.data.get_geom_jacp
                jacr = self.sim.data.get_geom_jacr
            elif object_type == "site":
                jacp = self.sim.data.get_site_jacp
                jacr = self.sim.data.get_site_jacr
            else:
                raise Exception("Invalid object type specified: ", object_type)

            jacp(name, self._J3NP)[self.jac_indices]  # pylint: disable=W0106
            jacr(name, self._J3NR)[self.jac_indices]  # pylint: disable=W0106

        # get the position Jacobian hstacked (1 x N_JOINTS*3)
        self._J6N[:3] = self._J3NP[self.jac_indices].reshape((3, self.N_JOINTS))
        # get the rotation Jacobian hstacked (1 x N_JOINTS*3)
        self._J6N[3:] = self._J3NR[self.jac_indices].reshape((3, self.N_JOINTS))

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return np.copy(self._J6N)

    def M(self, q=None):
        """Returns the inertia matrix in task space

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
        M = self._MNN_vector[self.M_indices]
        M = M.reshape((self.N_JOINTS, self.N_JOINTS))

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return np.copy(M)

    def R(self, name, q=None):
        """Returns the rotation matrix of the specified body

        Parameters
        ----------
        name: string
            The name of the Mujoco body to retrieve the Jacobian for
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        """
        if not self.use_sim_state and q is not None:
            old_q, old_dq, old_u = self._load_state(q)

        mjp.cymj._mju_quat2Mat(self._R9, self.sim.data.get_body_xquat(name))
        self._R = self._R9.reshape((3, 3))

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return self._R

    def quaternion(self, name, q=None):
        """Returns the quaternion of the specified body
        Parameters
        ----------

        name: string
            The name of the Mujoco body to retrieve the Jacobian for
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        """
        if not self.use_sim_state and q is not None:
            old_q, old_dq, old_u = self._load_state(q)

        quaternion = np.copy(self.sim.data.get_body_xquat(name))

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return quaternion

    def C(self, q=None, dq=None):
        """NOTE: The Coriolis and centrifugal effects (and gravity) are
        already accounted for by Mujoco in the qfrc_bias variable. There's
        no easy way to separate these, so all are returned by the g function.
        To prevent accounting for these effects twice, this function will
        return an error instead of qfrc_bias again.
        """
        raise NotImplementedError(
            "Coriolis and centrifugal effects already accounted "
            + "for in the term return by the gravity function."
        )

    def T(self, name, q=None, x=None):
        """Get the transform matrix of the specified body.

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

    def Tx(self, name, q=None, x=None, object_type="body"):
        """Returns the Cartesian coordinates of the specified Mujoco body

        Parameters
        ----------
        name: string
            The name of the Mujoco body to retrieve the Jacobian for
        q: float numpy.array, optional (Default: None)
            The joint angles of the robot. If None the current state is
            retrieved from the Mujoco simulator
        x: float numpy.array, optional (Default: None)
        object_type: string, the Mujoco object type, optional (Default: body)
            options: body, geom, site, camera, light, mocap
        """
        if x is not None and not np.allclose(x, 0):
            raise Exception("x offset currently not supported: ", x)

        if not self.use_sim_state and q is not None:
            old_q, old_dq, old_u = self._load_state(q)

        if object_type == "body":
            Tx = np.copy(self.sim.data.get_body_xpos(name))
        elif object_type == "geom":
            Tx = np.copy(self.sim.data.get_geom_xpos(name))
        elif object_type == "joint":
            Tx = np.copy(self.sim.data.get_joint_xanchor(name))
        elif object_type == "site":
            Tx = np.copy(self.sim.data.get_site_xpos(name))
        elif object_type == "camera":
            Tx = np.copy(self.sim.data.get_cam_xpos(name))
        elif object_type == "light":
            Tx = np.copy(self.sim.data.get_light_xpos(name))
        elif object_type == "mocap":
            Tx = np.copy(self.sim.data.get_mocap_pos(name))
        else:
            raise Exception("Invalid object type specified: ", object_type)

        if not self.use_sim_state and q is not None:
            self._load_state(old_q, old_dq, old_u)

        return Tx

    def T_inv(self, name, q=None, x=None):
        """Get the inverse transform matrix of the specified body.

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
