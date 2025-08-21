"""Isaac Sim Interface Module

This module provides an interface for controlling robots in Isaac Sim simulation environment.
It extends the base Interface class to provide Isaac Sim specific functionality.
"""

import numpy as np
from isaacsim import SimulationApp
from .interface import Interface

# Initialize simulation app before importing other Isaac Sim modules
simulation_app = SimulationApp({"headless": False})

import omni
import omni.kit.commands  # type: ignore
import isaacsim.core.utils.stage as stage_utils  # type: ignore
from omni.isaac.core import World  # type: ignore
from omni.isaac.core.articulations import ArticulationView  # type: ignore
from isaacsim.core.api.robots import Robot  # type: ignore
from pxr import UsdGeom, Gf, UsdShade, Sdf, UsdPhysics  # type: ignore
from isaacsim.core.utils.nucleus import get_assets_root_path  # type: ignore
import isaacsim.core.utils.numpy.rotations as rot_utils  # type: ignore


class IsaacSim(Interface):
    """An interface for Isaac Sim simulation environment.

    This class provides methods to connect to Isaac Sim, control robots,
    and interact with the simulation environment.

    Parameters
    ----------
    robot_config : object
        Configuration object containing all relevant information about the robot
        such as: number of joints, number of links, mass information, etc.
    dt : float, optional
        Simulation time step in seconds. Default is 0.001.

    Attributes
    ----------
    robot_config : object
        The robot configuration object
    dt : float
        Simulation time step
    prim_path : str
        USD prim path for the robot in the scene
    world : World
        Isaac Sim world object
    articulation_view : ArticulationView
        View of the robot articulation
    dof_indices : list
        Indices of degrees of freedom being controlled
    joint_indices : list
        Indices of joints being controlled
    """

    def __init__(self, robot_config, dt=0.001):
        """Initialize the Isaac Sim interface.
        
        Parameters
        ----------
        robot_config : object
            Robot configuration object
        dt : float, optional
            Simulation time step in seconds. Default is 0.001.
        """
        super().__init__(robot_config)
        self.robot_config = robot_config
        self.dt = dt
        self.prim_path = "/World/robot"

        # Initialize attributes that will be set during connection
        self.world = None
        self.articulation_view = None
        self.context = None
        self.stage = None
        self.dof_indices = []
        self.joint_indices = []
        self.all_dof_names = []
        self.all_joint_names = []
        self.all_body_names = []

    def connect(self, joint_names=None):
        """Connect to the Isaac Sim environment and set up the robot.

        Parameters
        ----------
        joint_names : list, optional
            List of joint names to send control signals to and get feedback from.
            If None, the joints in the kinematic tree connecting the end-effector
            to the world are used. Default is None.
        
        Raises
        ------
        Exception
            If a specified joint name does not exist in the robot model.
        """
        # Initialize the simulation world
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=self.dt,
            rendering_dt=self.dt
        )
        self.world.scene.add_default_ground_plane()
        self.context = omni.usd.get_context()
        self.stage = self.context.get_stage()

        # Load the robot from USD file
        assets_root_path = get_assets_root_path()
        robot_usd_path = f"{assets_root_path}{self.robot_config.robot_path}"
        print(f"Robot '{self.robot_config.robot_type}' is loaded from USD path: {robot_usd_path}")

        # Add robot to the stage
        stage_utils.add_reference_to_stage(
            usd_path=robot_usd_path,
            prim_path=self.prim_path,
        )
        robot = self.world.scene.add(
            Robot(prim_path=self.prim_path, name=self.robot_config.robot_type)
        )

        # Reset world to ensure proper initialization
        self.world.reset()

        # Create and initialize articulation view
        self.articulation_view = ArticulationView(
            prim_paths_expr=self.prim_path,
            name=self.robot_config.robot_type + "_view"
        )
        self.world.scene.add(self.articulation_view)
        self.articulation_view.initialize()

        # Add virtual end-effector if robot doesn't have one
        if not self.robot_config.has_EE:
            print("Robot has no EE, virtual one is attached.")
            self.add_virtual_ee_link(
                self.robot_config.EE_parent_link,
                self.robot_config.ee_link_name,
                offset=self.robot_config.ee_offset
            )

        # Reset the world to initialize physics
        self.world.reset()

        # Set up joint information
        self._setup_joint_indices(joint_names)

        # Connect robot config with simulation data
        print("Connecting to robot config...")
        self.robot_config._connect(
            self.world,
            self.stage,
            self.articulation_view,
            self.dof_indices,
            self.joint_indices,
            self.prim_path,
        )

        # Additional setup for mobile robots
        if not self.robot_config._is_fixed_base:
            self.world.add_physics_callback("keep_standing", self.keep_standing)
            # Adapt max force for better reactivity
            for joint_name in self.robot_config.controlled_dof:
                self.set_max_force(name=joint_name, value=0.7)
        else:
            self.set_gains()

    def _setup_joint_indices(self, joint_names):
        """Set up joint and DOF indices for control.
        
        Parameters
        ----------
        joint_names : list or None
            List of joint names to control
        """
        self.dof_indices = []
        self.joint_indices = []

        if joint_names is None:
            print("No joint names provided, using all controllable joints.")
            joint_names = self.robot_config.controlled_dof

        # Get all available names
        self.all_dof_names = self.articulation_view.dof_names
        self.all_joint_names = self.articulation_view.joint_names
        self.all_body_names = self.articulation_view.body_names

        # Map joint names to indices
        for name in joint_names:
            if name not in self.all_dof_names or name not in self.all_joint_names:
                raise Exception(f"Joint name {name} does not exist in robot model")
            
            joint_idx = self.articulation_view.get_joint_index(name)
            dof_idx = self.articulation_view.get_dof_index(name)
            self.dof_indices.append(dof_idx)
            self.joint_indices.append(joint_idx)

    def disconnect(self):
        """Disconnect from Isaac Sim and clean up resources.
        
        Properly closes the simulation application and cleans up any
        remaining connections or callbacks.
        """
        if hasattr(self, 'simulation_app') and self.simulation_app:
            self.simulation_app.close()
        print("IsaacSim connection closed...")

    def send_forces(self, u):
        """Apply torques to the controlled degrees of freedom.

        Parameters
        ----------
        u : numpy.ndarray
            Array of forces/torques to apply to the controlled DOFs.
            Length must match the number of controlled joints.
        """
        # Create full torque vector for all DOFs
        full_torques = np.zeros(self.robot_config.N_ALL_DOF)
        # Apply control torques to the controlled joints
        full_torques[self.dof_indices] = u
        # Apply the control signal
        self.articulation_view.set_joint_efforts(full_torques)
        # Move simulation ahead one time step
        self.world.step(render=True)

    def send_target_angles(self, q):
        """Move the robot to specified joint angles.

        Parameters
        ----------
        q : numpy.ndarray
            Array of target joint angles in radians.
            Length must match the number of controlled joints.
        """
        self.robot_config._set_joint_positions(q)
        self.world.step(render=True)

    def get_feedback(self):
        """Get current robot state information.

        Returns
        -------
        dict
            Dictionary containing:
            - 'q': Joint angles in radians
            - 'dq': Joint velocities in rad/sec
        """
        self.q = self.robot_config._get_joint_positions()
        self.dq = self.robot_config._get_joint_velocities()
        return {"q": self.q, "dq": self.dq}

    def set_xyz(self, name, xyz):
        """Set the position of an object in the environment.

        Parameters
        ----------
        name : str
            The prim path of the object to move
        xyz : numpy.ndarray
            The [x, y, z] location of the target in meters
        """
        prim = self.robot_config._get_prim(name)
        xformable = UsdGeom.Xformable(prim)
        transform_matrix = Gf.Matrix4d().SetTranslate(
            Gf.Vec3d(xyz[0], xyz[1], xyz[2])
        )
        xformable.MakeMatrixXform().Set(transform_matrix)

    def set_orientation(self, name, quat_wxyz):
        """Set the orientation of an object in the environment.

        Parameters
        ----------
        name : str
            The prim path of the object to rotate
        quat_wxyz : numpy.ndarray
            The [w, x, y, z] quaternion representation of the target orientation
        """
        prim = self.robot_config._get_prim(name)
        xformable = UsdGeom.Xformable(prim)
        quat = Gf.Quatd(
            quat_wxyz[0],
            Gf.Vec3d(quat_wxyz[1], quat_wxyz[2], quat_wxyz[3])
        )
        transform_matrix = Gf.Matrix4d().SetRotate(quat)
        xformable.MakeMatrixXform().Set(transform_matrix)

    def keep_standing(self, dt):
        """Physics callback to keep humanoid robots upright.
        
        This method is used as a physics callback to prevent humanoid
        robots from falling over during simulation.

        Parameters
        ----------
        dt : float
            Time step (automatically passed by physics callback system)
        """
        prim = self.stage.GetPrimAtPath(self.robot_config.lock_prim_standing)
        prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
        prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.0, 1.4))

    def create_target_prim(self, prim_path="/World/target", size=0.1, 
                          color=np.array([0, 0, 1.0])):
        """Create a visual-only target cube in the scene.

        Parameters
        ----------
        prim_path : str, optional
            USD prim path for the target object. Default is "/World/target".
        size : float, optional
            Size of the cube in meters. Default is 0.1.
        color : numpy.ndarray, optional
            RGB color values [0-1]. Default is blue [0, 0, 1.0].

        Returns
        -------
        UsdGeom.Cube
            The created cube primitive
        """
        # Create cube geometry
        cube_prim = UsdGeom.Cube.Define(self.stage, prim_path)
        cube_prim.CreateSizeAttr(size)

        # Create and apply material for color
        material_path = prim_path + "/Material"
        material = UsdShade.Material.Define(self.stage, material_path)

        # Create shader
        shader = UsdShade.Shader.Define(self.stage, material_path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(color[0], color[1], color[2])
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

        # Connect shader to material and bind material to cube
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        UsdShade.MaterialBindingAPI(cube_prim).Bind(material)

        # Disable collision to ensure it's purely visual
        cube_prim.GetPrim().CreateAttribute(
            "physics:collisionEnabled", Sdf.ValueTypeNames.Bool
        ).Set(False)
        
        return cube_prim

    def create_random_pos(self):
        """Generate a random position within the robot's target bounds.

        Returns
        -------
        numpy.ndarray
            Random [x, y, z] position within the configured target bounds
        """
        pos_target = np.array([
            np.random.uniform(
                low=self.robot_config.target_min[0],
                high=self.robot_config.target_max[0]
            ),
            np.random.uniform(
                low=self.robot_config.target_min[1],
                high=self.robot_config.target_max[1]
            ),
            np.random.uniform(
                low=self.robot_config.target_min[2],
                high=self.robot_config.target_max[2]
            ),
        ])
        return pos_target

    def set_max_force(self, name, value):
        """Set maximum force for a joint using PhysX DriveAPI.

        Parameters
        ----------
        name : str
            Name of the joint to configure
        value : float
            Maximum force value to set
        """
        prim = self.robot_config._get_prim(name)
        # Apply the DriveAPI if not already present
        # Use "linear" for prismatic joints, "angular" for revolute joints
        driveAPI = UsdPhysics.DriveAPI.Apply(prim, "angular")
        # Set the max force
        driveAPI.CreateMaxForceAttr(value)

    def set_gains(self):
        """Set PD controller gains for robot joints.
        
        Sets appropriate stiffness and damping values for controlled
        and non-controlled joints. Controlled joints get low stiffness
        for force control, while other joints maintain position.
        """
        # Initialize with default high stiffness for position holding
        stiffness = np.ones(self.robot_config.N_ALL_DOF) * 100.0
        damping = np.ones(self.robot_config.N_ALL_DOF) * 10.0

        # Set low gains for controlled joints (force control)
        for idx in self.dof_indices:
            stiffness[idx] = 0.0
            damping[idx] = 0.1

        self.articulation_view.set_gains(stiffness, damping)

    def add_virtual_ee_link(self, EE_parent_link, ee_name, offset):
        """Add a virtual end-effector link as an Xform under the parent link.

        This method creates a virtual end-effector when the robot model
        doesn't include one explicitly.

        Parameters
        ----------
        EE_parent_link : str
            Name of the parent link to attach the virtual EE to
        ee_name : str
            Name for the virtual end-effector link
        offset : numpy.ndarray or list
            [x, y, z] offset from parent link in meters
        """
        parent_path = f"{self.prim_path}/{EE_parent_link}"
        # Full path to the new EE transform, nested under parent
        ee_prim_path = f"{parent_path}/{ee_name}"
        
        # Create the Xform prim
        ee_prim = UsdGeom.Xform.Define(self.stage, ee_prim_path)
        
        # Set transform relative to parent
        ee_prim.AddTranslateOp().Set(Gf.Vec3d(*offset))