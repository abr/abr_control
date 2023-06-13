import glfw
import mujoco
import mujoco_viewer
import numpy as np

from abr_control.utils import transformations

from .interface import Interface

class Mujoco(Interface):
    """An interface for MuJoCo.

    Parameters
    ----------
    robot_config: class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    dt: float, optional (Default: 0.001)
        simulation time step in seconds
    visualize: boolean, optional (Default: True)
        turns visualization on or off
    create_offscreen_rendercontext: boolean, optional (Default: False)
        create the offscreen rendercontext behind the main visualizer
        (helpful for rendering images from other cameras without displaying them)
    """

    def __init__(
        self,
        robot_config,
        dt=0.001,
        visualize=True,
        create_offscreen_rendercontext=False,
    ):

        super().__init__(robot_config)

        self.dt = dt  # time step
        self.count = 0  # keep track of how many times send forces is called

        self.robot_config = robot_config

        # turns the visualization on or off
        self.visualize = visualize
        # if we want the offscreen render context
        self.create_offscreen_rendercontext = create_offscreen_rendercontext

    def connect(self, joint_names=None, camera_id=-1):
        """
        joint_names: list, optional (Default: None)
            list of joint names to send control signal to and get feedback from
            if None, the joints in the kinematic tree connecting the end-effector
            to the world are used
        camera_id: int, optional (Default: -1)
            the id of the camera to use for the visualization
        """
        self.model = mujoco.MjModel.from_xml_path(self.robot_config.xml_file)
        self.data = mujoco.MjData(self.model)
        # set the time step for simulation
        self.model.opt.timestep = self.dt

        mujoco.mj_forward(self.model, self.data)  # run forward to fill in sim.data

        self.joint_pos_addrs = []
        self.joint_dyn_addrs = []

        if joint_names is None:
            # if no joint names provided, get addresses of joints in the kinematic
            # tree from end-effector (EE) to world body
            bodyid = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                "EE"
            )
            # and working back to the world body
            while self.model.body_parentid[bodyid] != 0:
                first_joint = self.model.body_jntadr[bodyid]
                num_joints = self.model.body_jntnum[bodyid]

                for jntadr in range(first_joint, first_joint + num_joints):
                    self.joint_pos_addrs += self.get_joint_pos_addrs(jntadr)
                    self.joint_dyn_addrs += self.get_joint_dyn_addrs(jntadr)
                bodyid = self.model.body_parentid[bodyid]

            self.joint_pos_addrs = self.joint_pos_addrs[::-1]
            self.joint_dyn_addrs = self.joint_dyn_addrs[::-1]

        else:
            for name in joint_names:
                jntadr = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    name
                )
                self.joint_pos_addrs += self.get_joint_pos_addrs(jntadr)
                self.joint_dyn_addrs += self.get_joint_dyn_addrs(jntadr)

        # give the robot config access to the sim for wrapping the
        # forward kinematics / dynamics functions
        self.robot_config._connect(
            self.model,
            self.data,
            self.joint_pos_addrs,
            self.joint_dyn_addrs,
        )

        # if we want to use the offscreen render context create it before the
        # viewer so the corresponding window is behind the viewer
        if self.create_offscreen_rendercontext:
            self.offscreen = mujoco.MjRenderContextOffscreen(self.sim, 0)

        # create the visualizer
        if self.visualize:
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
            # set the default display to skip frames to speed things up
            self.viewer._render_every_frame = False

        print("MuJoCo session created")

    def disconnect(self):
        """Stop and reset the simulation
        """
        if self.visualize:
            self.viewer.close()

        print("MuJoCO session closed...")

    def get_joint_pos_addrs(self, jntadr):
        # store the data.qpos indices associated with this joint
        first_pos = self.model.jnt_qposadr[jntadr]
        posvec_length = self.robot_config.JNT_POS_LENGTH[self.model.jnt_type[jntadr]]
        joint_pos_addr = list(range(first_pos, first_pos + posvec_length))[::-1]
        return joint_pos_addr

    def get_joint_dyn_addrs(self, jntadr):
        # store the data.qvel and .ctrl indices associated with this joint
        first_dyn = self.model.jnt_dofadr[jntadr]
        dynvec_length = self.robot_config.JNT_DYN_LENGTH[self.model.jnt_type[jntadr]]
        joint_dyn_addr = list(range(first_dyn, first_dyn + dynvec_length))[::-1]
        return joint_dyn_addr

    def send_forces(self, u, update_display=True, use_joint_dyn_addrs=True):
        """Apply the specified torque to the robot joints, move the simulation
        one time step forward, and update the position of the hand object.

        Parameters
        ----------
        u: np.array
            the torques to apply to the robot [Nm]
        update_display: boolean, Optional (Default:True)
            toggle for updating display
        use_joint_dyn_addrs: boolean
            set false to update the control signal for all actuators
        """
        if use_joint_dyn_addrs:
            self.data.ctrl[self.joint_dyn_addrs] = u[:]
        else:
            self.data.ctrl[:] = u[:]

        # move simulation ahead one time step
        mujoco.mj_step(self.model, self.data)

        # Update position of hand object
        feedback = self.get_feedback()
        hand_xyz = self.robot_config.Tx(name="EE", q=feedback["q"])
        self.set_mocap_xyz("hand", hand_xyz)

        # Update orientation of hand object
        hand_quat = self.robot_config.quaternion(name="EE", q=feedback["q"])
        self.set_mocap_orientation("hand", hand_quat)

        if self.visualize and update_display:
            self.viewer.render()

        self.count += self.dt

    def set_external_force(self, name, u_ext):
        """Applies an external force to a specified body

        Parameters
        ----------
        u_ext: np.array([x, y, z, alpha, beta, gamma])
            external force to apply [Nm]
        name: string
            name of the body to apply the force to
        """
        self.sim.data.xfrc_applied[self.model.body_name2id(name)] = u_ext

    def send_target_angles(self, q, use_joint_pos_addrs=True):
        """Move the robot to the specified configuration.

        Parameters
        ----------
        q: np.array
            configuration to move to [radians]
        """

        if use_joint_pos_addrs:
            self.data.qpos[self.joint_pos_addrs] = np.copy(q)
        else:
            self.data.qpos[:] = q[:]
        mujoco.mj_forward(self.model, self.data)

    def set_joint_state(self, q, dq):
        """Move the robot to the specified configuration.

        Parameters
        ----------
        q: np.array
            configuration to move to [rad]
        dq: np.array
            joint velocities [rad/s]
        """

        self.data.qpos[self.joint_pos_addrs] = np.copy(q)
        self.data.qvel[self.joint_dyn_addrs] = np.copy(dq)
        mujoco.mj_forward(self.model, self.data)

    def get_feedback(self):
        """Returns the joint angles and joint velocities in [rad] and [rad/sec],
        respectively, in a dictionary.
        """

        self.q = np.copy(self.data.qpos[self.joint_pos_addrs])
        self.dq = np.copy(self.data.qvel[self.joint_dyn_addrs])

        return {"q": self.q, "dq": self.dq}

    def get_xyz(self, name, object_type="body"):
        """Returns the xyz position of the specified object

        name: string
            name of the object you want the xyz position of
        object_type: string
            type of object you want the xyz position of
            Can be: body, geom, site
        """
        if object_type == "body":
            xyz = self.data.body(name).xpos
        elif object_type == "geom":
            xyz = self.data.geom(name).xpos
        elif object_type == "site":
            xyz = self.data.site(name).xpos
            xyz = self.sim.data.get_site_xpos(name)
        elif object_type == "camera":
            xyz = self.sim.data.get_camera_xpos(name)
        else:
            raise Exception(f"get_xyz for {object_type} object type not supported")

        return np.copy(xyz)

    def get_orientation(self, name, object_type="body"):
        """Returns the orientation of an object as the [w x y z] quaternion [radians]

        Parameters
        ----------
        name: string
            the name of the object of interest
        object_type: string, Optional (Default: body)
            The type of mujoco object to get the orientation of.
            Can be: body, geom, site
        """
        if object_type == "mocap":  # commonly queried to find target
            quat = self.sim.data.get_mocap_quat(name)
        elif object_type == "body":
            quat = self.data.body(name).xquat
        elif object_type == "geom":
            xmat = self.data.geom(name).xmat
            quat = transformations.quaternion_from_matrix(xmat.reshape((3, 3)))
        elif object_type == "site":
            xmat = self.data.site(name).xmat
            quat = transformations.quaternion_from_matrix(xmat.reshape((3, 3)))
        elif object_type == "camera":
            xmat = self.sim.data.get_camera_xmat(name)
            quat = transformations.quaternion_from_matrix(xmat.reshape((3, 3)))
        else:
            raise Exception(
                f"get_orientation for {object_type} object type not supported"
            )
        return np.copy(quat)

    def set_mocap_xyz(self, name, xyz):
        """Set the position of a mocap object in the Mujoco environment.

        name: string
            the name of the object
        xyz: np.array
            the [x,y,z] location of the target [meters]
        """
        mocap_id = self.model.body(name).mocapid
        self.data.mocap_pos[mocap_id] = xyz
        mujoco.mj_forward(self.model, self.data)

    def set_mocap_orientation(self, name, quat):
        """Sets the orientation of an object in the Mujoco environment

        Sets the orientation of an object using the provided Euler angles.
        Angles must be in a relative xyz frame.

        Parameters
        ----------
        name: string
            the name of the object of interest
        quat: np.array
            the [w x y z] quaternion [radians] for the object.
        """
        mocap_id = self.model.body(name).mocapid
        self.data.mocap_quat[mocap_id] = quat
        mujoco.mj_forward(self.model, self.data)

    def set_state(self, name, xyz=None, quat=None):
        """Sets the state of an object attached to the world with a free joint.

        Parameters
        ----------
        name: string
            the name of the object of interest
        xyz: np.array
            the [x,y,z] location of the target [meters]
        quat: np.array
            the [w x y z] quaternion [radians] for the object.
        """
        assert (xyz is not None) or (quat is not None)

        # get the address of the joint attached to the body
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        jnt_adr = self.model.body_jntadr[body_id]
        # confirm that it's a free joint
        assert self.model.jnt_type[jnt_adr] == mujoco.mjtJoint.mjJNT_FREE

        # get the address of the joint angles in the data.qpos array
        jnt_qpos_adr = self.model.jnt_qposadr[jnt_adr]
        if xyz is not None:
            # set the new position
            self.data.qpos[jnt_qpos_adr:jnt_qpos_adr + 3] = xyz
        if quat is not None:
            # set the new orientation
            self.data.qpos[jnt_qpos_adr + 3:jnt_qpos_adr + 7] = quat

        # run mj_forward to propogate the change immediately
        mujoco.mj_forward(self.model, self.data)


