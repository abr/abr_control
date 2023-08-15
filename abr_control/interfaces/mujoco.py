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
    display_frequency: int, optional (Default: 1)
        How often to render the frame to display on screen.
        EX:
            a value of 1 displays every sim frame
            a value of 5 displays every 5th frame
            a value of 0 runs the simulation offscreen
    render_params : dict, optional (Default: None)
        'cameras' : list
            camera ids, the order the camera output is appended in
        'resolution' : list
            the resolution to return
        'update_frequency' : int, optional (Default: 1)
            How often to render the image
        Used to render offscreen cameras for RGB camera feedback
    """

    def __init__(
        self,
        robot_config,
        dt=0.001,
        display_frequency=1,
        render_params=None
    ):
        super().__init__(robot_config)

        self.robot_config = robot_config
        self.dt = dt  # time step
        self.display_frequency = display_frequency
        self.render_params = render_params
        self.count = 0  # keep track of how many times send forces is called
        self.timestep = 0

        self.camera_feedback = np.array([])
        if self.render_params:
            # if not empty, create array for storing rendered camera feedback
            self.camera_feedback = np.zeros(
                (
                    self.render_params["resolution"][0],
                    self.render_params["resolution"][1]
                    * len(self.render_params["cameras"]),
                    3,
                )
            )
            self.subpixels = np.product(self.camera_feedback.shape)
            if "frequency" not in self.render_params.keys():
                self.render_params["frequency"] = 1



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
        self.joint_vel_addrs = []
        self.joint_dyn_addrs = []

        if joint_names is None:
            # if no joint names provided, get addresses of joints in the kinematic
            # tree from end-effector (EE) to world body
            bodyid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "EE")
            # and working back to the world body
            while self.model.body_parentid[bodyid] != 0:
                first_joint = self.model.body_jntadr[bodyid]
                num_joints = self.model.body_jntnum[bodyid]

                for jntadr in range(first_joint, first_joint + num_joints):
                    self.joint_pos_addrs += self.get_joint_pos_addrs(jntadr)
                    self.joint_vel_addrs += self.get_joint_vel_addrs(jntadr)
                    self.joint_dyn_addrs += self.get_joint_dyn_addrs(jntadr)
                bodyid = self.model.body_parentid[bodyid]

            self.joint_pos_addrs = self.joint_pos_addrs[::-1]
            self.joint_vel_addrs = self.joint_vel_addrs[::-1]
            self.joint_dyn_addrs = self.joint_dyn_addrs[::-1]

        else:
            for name in joint_names:
                jntadr = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                self.joint_pos_addrs += self.get_joint_pos_addrs(jntadr)[::-1]
                self.joint_vel_addrs += self.get_joint_vel_addrs(jntadr)[::-1]
                self.joint_dyn_addrs += self.get_joint_dyn_addrs(jntadr)[::-1]

        # give the robot config access to the sim for wrapping the
        # forward kinematics / dynamics functions
        print("Connecting to robot config...")
        self.robot_config._connect(
            self.model,
            self.data,
            self.joint_pos_addrs,
            self.joint_vel_addrs,
        )

        # create the visualizer
        if self.display_frequency > 0:
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
            # set the default display to skip frames to speed things up
            self.viewer._render_every_frame = False

            if camera_id > -1:
                self.viewer.cam.fixedcamid = camera_id
                self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED

        if self.render_params is not None:
            self.offscreen = mujoco_viewer.MujocoViewer(
                self.model,
                self.data,
                height=self.render_params["resolution"][1],
                width=self.render_params["resolution"][0],
                mode="offscreen",
            )
            # set the default display to skip frames to speed things up
            self.offscreen._render_every_frame = False

        glfw.make_context_current(self.viewer.window)
        print("MuJoCo session created")

    def disconnect(self):
        """Stop and reset the simulation"""
        if self.display_frequency > 0:
            self.viewer.close()

        print("MuJoCO session closed...")

    def get_joint_pos_addrs(self, jntadr):
        # store the data.qpos indices associated with this joint
        first_pos = self.model.jnt_qposadr[jntadr]
        posvec_length = self.robot_config.JNT_POS_LENGTH[self.model.jnt_type[jntadr]]
        joint_pos_addr = list(range(first_pos, first_pos + posvec_length))[::-1]
        return joint_pos_addr

    def get_joint_vel_addrs(self, jntadr):
        # store the data.qvel indices associated with this joint
        first_vel = self.model.jnt_dofadr[jntadr]
        velvec_length = self.robot_config.JNT_DYN_LENGTH[self.model.jnt_type[jntadr]]
        joint_vel_addr = list(range(first_vel, first_vel + velvec_length))[::-1]
        return joint_vel_addr

    def get_joint_dyn_addrs(self, jntadr):
        # store the .ctrl indices associated with this joint
        for first_dyn, v in enumerate(self.model.actuator_trnid):
            if v[0] == jntadr:
                break
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

        self.count += self.dt
        self.timestep = int(self.count / self.dt)

        freq_display = not self.timestep % self.display_frequency

        if freq_display and update_display:
            self.viewer.render()

    def set_external_force(self, name, u_ext):
        """Applies an external force to a specified body

        Parameters
        ----------
        u_ext: np.array([x, y, z, alpha, beta, gamma])
            external force to apply [Nm]
        name: string
            name of the body to apply the force to
        """
        bodyid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "EE")
        self.data.xfrc_applied[bodyid] = u_ext

    def send_target_angles(self, q):
        """Move the robot to the specified configuration.

        Parameters
        ----------
        q: np.array
            configuration to move to [radians]
        """

        self.data.qpos[self.joint_pos_addrs] = np.copy(q)
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
        self.data.qvel[self.joint_vel_addrs] = np.copy(dq)
        mujoco.mj_forward(self.model, self.data)

    def get_feedback(self):
        """Returns the joint angles and joint velocities in [rad] and [rad/sec],
        respectively, in a dictionary.
        """

        self.q = np.copy(self.data.qpos[self.joint_pos_addrs])
        self.dq = np.copy(self.data.qvel[self.joint_vel_addrs])

        if self.render_params:
            res = self.render_params['resolution']
            if self.timestep % self.render_params["frequency"] == 0:
                for ii, jj in enumerate(self.render_params["cameras"]):
                    glfw.make_context_current(self.offscreen.window)
                    self.camera_feedback[
                        :, ii * res[1] : (ii + 1) * res[1]
                    ] = self.offscreen.read_pixels(camid=jj)

                    glfw.make_context_current(None)

            glfw.make_context_current(self.viewer.window)

        return {"q": self.q, "dq": self.dq, "rgb": self.camera_feedback}

    def get_xyz(self, name, object_type="body"):
        """Returns the xyz position of the specified object

        name: string
            name of the object you want the xyz position of
        object_type: string
            type of object you want the xyz position of
            Can be: body, geom, site
        """
        if object_type == "mocap":  # commonly queried to find target
            mocap_id = self.model.body(name).mocapid
            xyz = self.data.mocap_pos[mocap_id]
        elif object_type == "body":
            xyz = self.data.body(name).xpos
        elif object_type == "geom":
            xyz = self.data.geom(name).xpos
        elif object_type == "site":
            xyz = self.data.site(name).xpos
        elif object_type == "camera":
            xyz = self.data.camera(name).xpos
        elif object_type == "joint":
            xyz = self.model.jnt(name).pos
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
            mocap_id = self.model.body(name).mocapid
            quat = self.data.mocap_quat[mocap_id]
        elif object_type == "body":
            quat = self.data.body(name).xquat
        elif object_type == "geom":
            xmat = self.data.geom(name).xmat
            quat = transformations.quaternion_from_matrix(xmat.reshape((3, 3)))
        elif object_type == "site":
            xmat = self.data.site(name).xmat
            quat = transformations.quaternion_from_matrix(xmat.reshape((3, 3)))
        elif object_type == "camera":
            xmat = self.data.camera(name).xmat
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
            self.data.qpos[jnt_qpos_adr : jnt_qpos_adr + 3] = xyz
        if quat is not None:
            # set the new orientation
            self.data.qpos[jnt_qpos_adr + 3 : jnt_qpos_adr + 7] = quat

        # run mj_forward to propogate the change immediately
        mujoco.mj_forward(self.model, self.data)
