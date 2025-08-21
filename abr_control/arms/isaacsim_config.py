"""Isaac Sim Configuration Module

This module provides robot configuration for Isaac Sim simulation environment.
It handles robot-specific parameters, kinematics, and dynamics calculations.
"""

import numpy as np
import omni


class IsaacsimConfig:
    """A wrapper for Isaac Sim simulator to generate kinematics and dynamics calculations.
    
    This class provides robot-specific configurations and interfaces with Isaac Sim
    to compute forward kinematics, Jacobians, mass matrices, and other dynamics
    properties necessary for robot controllers.

    Parameters
    ----------
    robot_type : str
        The name of the robot model to load. Supported robots:
        - "ur5": Universal Robots UR5 manipulator
        - "jaco2": Kinova Jaco2 manipulator  
        - "h1": Unitree H1 humanoid robot
        - "h1_hands": Unitree H1 humanoid with hands

    Attributes
    ----------
    robot_type : str
        Name of the robot type
    env_idx : int
        Environment index for multi-environment simulations
    ee_link_name : str
        Name of the end-effector link
    robot_path : str
        USD file path for the robot model
    has_EE : bool
        Whether robot has a physical end-effector
    EE_parent_link : str
        Name of the parent link for end-effector attachment
    ee_offset : list
        Offset from parent link to end-effector [x, y, z]
    target_min : numpy.ndarray
        Minimum bounds for target workspace [x, y, z]
    target_max : numpy.ndarray
        Maximum bounds for target workspace [x, y, z]  
    controlled_dof : list
        List of controlled joint names
    START_ANGLES : numpy.ndarray
        Default starting joint angles in radians
    """

    def __init__(self, robot_type):
        """Initialize robot configuration for specified robot type.

        Parameters
        ----------
        robot_type : str
            The name of the robot model to load
            
        Raises
        ------
        ValueError
            If robot_type is not supported
        """
        self.robot_type = robot_type
        self.env_idx = 0  # Assuming single robot environment
        self.ee_link_name = "EE"  # Default name for virtual end-effector
        
        # Initialize robot-specific configurations
        self._configure_robot()
        
        # Convert START_ANGLES string to numpy array
        if hasattr(self, '_start_angles_str'):
            self.START_ANGLES = np.array(self._start_angles_str.split(), dtype=float)

    def _configure_robot(self):
        """Configure robot-specific parameters based on robot type."""
        if self.robot_type == "ur5":
            self._configure_ur5()
        elif self.robot_type == "jaco2":
            self._configure_jaco2()
        elif self.robot_type == "h1":
            self._configure_h1()
        elif self.robot_type == "h1_hands":
            self._configure_h1_hands()
        else:
            raise ValueError(f"Unsupported robot type: {self.robot_type}")

    def _configure_ur5(self):
        """Configure parameters for UR5 robot."""
        self.robot_path = "/Isaac/Robots/UniversalRobots/ur5/ur5.usd"
        self.has_EE = False  # UR5 has no end-effector
        self.EE_parent_link = "wrist_3_link"
        self.ee_offset = [0.0, 0.0, 0.0]
        self._start_angles_str = "0 -.67 -.67 0 0 0"
        self.target_min = np.array([-0.4, -0.4, 0.3])
        self.target_max = np.array([0.4, 0.4, 0.6])
        self.controlled_dof = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        print(f"Virtual end effector '{self.ee_link_name}' will be attached "
              f"(robot has no physical EE).")

    def _configure_jaco2(self):
        """Configure parameters for Jaco2 robot."""
        self.robot_path = "/Isaac/Robots/Kinova/Jaco2/J2N6S300/j2n6s300_instanceable.usd"
        self.has_EE = True  # Jaco2 has an end-effector
        self.EE_parent_link = "j2n6s300_link_6"
        self.ee_link_name = "j2n6s300_end_effector"
        self._start_angles_str = "2.0 3.14 1.57 4.71 0.0 3.04"
        self.target_min = np.array([-0.4, -0.4, 0.3])
        self.target_max = np.array([0.4, 0.4, 0.6])
        self.controlled_dof = [
            'j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3',
            'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6'
        ]
        print(f"End effector '{self.ee_link_name}' found in USD, using existing EE.")

    def _configure_h1(self):
        """Configure parameters for H1 humanoid robot."""
        self.robot_path = "/Isaac/Robots/Unitree/H1/h1.usd"
        self.has_EE = False  # H1 has no end-effector
        self.EE_parent_link = "right_elbow_link"
        self.ee_offset = [0.26455, 0.00118, -0.0209]  # H1 specific offset
        self._start_angles_str = "0. 0. 0. 0."
        self.target_min = np.array([0.12, -0.4, 1.4])
        self.target_max = np.array([0.4, 0.05, 2.2])
        self.controlled_dof = [
            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint', 'right_elbow_joint'
        ]
        self.lock_prim_standing = '/World/robot/pelvis'
        print(f"Virtual end effector '{self.ee_link_name}' will be attached "
              f"(robot has no physical EE).")

    def _configure_h1_hands(self):
        """Configure parameters for H1 humanoid robot with hands."""
        self.robot_path = "/Isaac/Robots/Unitree/H1/h1_with_hand.usd"
        self.has_EE = True  # H1 with hands has an end-effector
        self.EE_parent_link = "right_elbow_link"
        self.ee_link_name = "right_hand_link"
        self._start_angles_str = "0. 0. 0. 0. 0."
        self.target_min = np.array([0.12, -0.4, 1.4])
        self.target_max = np.array([0.49, 0.05, 2.2])
        self.controlled_dof = [
            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_hand_joint'
        ]
        self.lock_prim_standing = '/World/robot/pelvis'
        print(f"End effector '{self.ee_link_name}' found in USD, using existing EE.")

    def _connect(self, world, stage, articulation_view, dof_indices, 
                 joint_indices, prim_path):
        """Connect the configuration to Isaac Sim simulation.
        
        Called by the interface once the Isaac Sim simulation is created,
        this connects the config to the simulator to access kinematics
        and dynamics information.
        
        Parameters
        ----------
        world : omni.isaac.core.World
            The Isaac Sim World object managing the simulation environment
        stage : pxr.Usd.Stage
            The USD stage containing the scene hierarchy and prims
        articulation_view : omni.isaac.core.articulations.ArticulationView
            ArticulationView providing access to robot's kinematic and
            dynamic properties in Isaac Sim
        dof_indices : numpy.ndarray
            Indices of controlled robot joints in the articulation's DOF array
        joint_indices : numpy.ndarray
            Indices of controlled robot joints in the articulation's joint array
        prim_path : str
            USD prim path to the robot articulation in scene hierarchy
        """
        # Store Isaac Sim interface objects
        self.world = world
        self.stage = stage
        self.articulation_view = articulation_view
        self.dof_indices = np.copy(dof_indices)
        self.joint_indices = np.copy(joint_indices)
        self.prim_path = prim_path
        
        # Robot configuration parameters
        self.N_JOINTS = len(self.dof_indices)
        self.N_ALL_DOF = self.articulation_view.num_dof
        self.N_ALL_JOINTS = self.articulation_view.num_joints
        self._is_fixed_base = self._check_fixed_base()
        
        # Offset for non-fixed base robots (mobile robots)
        # Necessary for correct indexing of Jacobian matrices
        self.base_offset = 0 if self._is_fixed_base else 6
        
        # Pre-compute indices for efficient matrix operations
        self._setup_matrix_indices()
        
        # Initialize data storage arrays
        self._initialize_data_arrays()

    def _check_fixed_base(self):
        """Check if robot has a fixed base or is mobile.
        
        Returns
        -------
        bool
            True if robot has fixed base, False if mobile
        """
        num_dof = self.articulation_view.num_dof
        jacobian_shape = self.articulation_view.get_jacobian_shape()[2]
        return num_dof == jacobian_shape

    def _setup_matrix_indices(self):
        """Pre-compute indices for efficient matrix operations."""
        # Jacobian indices for position and rotation (6DOF)
        self.jac_indices = np.hstack([
            self.dof_indices + (ii * self.N_ALL_DOF) for ii in range(6)
        ])
        
        # Mass matrix indices
        self.M_indices = [
            ii * self.N_ALL_DOF + jj
            for jj in self.dof_indices
            for ii in self.dof_indices
        ]

    def _initialize_data_arrays(self):
        """Initialize arrays for storing dynamics data."""
        self._g = np.zeros(self.N_JOINTS)
        self._J6N = np.zeros((6, self.N_JOINTS))
        self._MNN = np.zeros((self.N_ALL_DOF, self.N_ALL_DOF))
        self._R = np.zeros((3, 3))

    def g(self, q=None):
        """Get gravitational forces for controlled joints.

        Parameters
        ----------
        q : numpy.ndarray, optional
            Joint angles (not used in current implementation)
            
        Returns
        -------
        numpy.ndarray
            Gravitational forces for controlled joints
        """
        gravity_forces = self.articulation_view.get_generalized_gravity_forces()
        self._g = gravity_forces[self.env_idx]
        
        if self._is_fixed_base:
            return -self._g[self.dof_indices]
        else:
            # For mobile robots, gravity compensation is typically handled differently
            return np.zeros(len(self.dof_indices))

    def dJ(self, name, q=None, dq=None, x=None):
        """Get derivative of Jacobian with respect to time.

        Parameters
        ----------
        name : str
            Name of the prim to compute derivative for
        q : numpy.ndarray, optional
            Joint angles
        dq : numpy.ndarray, optional
            Joint velocities
        x : numpy.ndarray, optional
            Additional state variables
            
        Raises
        ------
        NotImplementedError
            This method is not currently implemented
        """
        raise NotImplementedError("Jacobian derivative not implemented yet")

    def J(self, name, q=None, x=None):
        """Get Jacobian matrix for the specified end-effector.
        
        For mobile robots, the floating base is accounted for with an offset.

        Parameters
        ----------
        name : str
            Name of the prim to compute Jacobian for
        q : numpy.ndarray, optional
            Joint angles (not used in current implementation)
        x : numpy.ndarray, optional
            Additional state variables
            
        Returns
        -------
        numpy.ndarray
            6×N Jacobian matrix where N is number of controlled joints
            First 3 rows: linear velocity Jacobian
            Last 3 rows: angular velocity Jacobian
            
        Raises
        ------
        RuntimeError
            If ArticulationView contains no environments or Jacobian is all zeros
        """
        jacobians = self.articulation_view.get_jacobians(clone=True)
        
        if jacobians.shape[0] == 0:
            raise RuntimeError("ArticulationView contains no environments.")

        # Account for fixed vs mobile base indexing
        offset = -1 if self._is_fixed_base else 0
        body_index = self.articulation_view.get_body_index(self.EE_parent_link) + offset

        J_full = jacobians[self.env_idx, body_index, :, :]
        
        if len(jacobians.shape) == 4:
            # Apply offset for mobile robots and extract controlled DOF columns
            indices = self.dof_indices + self.base_offset
            J = J_full[:, indices]
        else:
            raise RuntimeError(f"Unexpected jacobians shape: {jacobians.shape}")

        # Store linear and angular velocity Jacobians
        self._J6N[:3, :] = J[:3, :]  # Linear velocity
        self._J6N[3:, :] = J[3:, :]  # Angular velocity

        if np.allclose(J, 0):
            raise RuntimeError(
                "Jacobian is all zeros - check robot configuration and joint addresses"
            )
            
        return np.copy(self._J6N)

    def M(self, q=None):
        """Get mass/inertia matrix for controlled joints.
        
        For mobile robots, the floating base is accounted for with an offset.

        Parameters
        ----------
        q : numpy.ndarray, optional
            Joint angles (not used in current implementation)
            
        Returns
        -------
        numpy.ndarray
            N×N mass matrix where N is number of controlled joints
        """
        # Get full mass matrix (num_envs, joint_count, joint_count)
        mass_matrices = self.articulation_view.get_mass_matrices()
        self._MNN = mass_matrices[self.env_idx]
        
        # Apply offset for mobile robots and extract controlled joints
        indices = self.dof_indices + self.base_offset
        M = self._MNN[np.ix_(indices, indices)]
        
        return M

    def R(self, name, q=None):
        """Get rotation matrix for the specified object.
        
        Parameters
        ----------
        name : str
            Name of the object (body, geom, or site) in USD stage
        q : numpy.ndarray, optional
            Joint positions (not used unless simulating different state)
            
        Returns
        -------
        numpy.ndarray
            3×3 rotation matrix
            
        Raises
        ------
        RuntimeError
            If prim is not found or invalid
        """
        prim = self._get_prim(name)
        if not prim.IsValid():
            raise RuntimeError(f"Prim '{name}' not found or invalid")

        # Get 4×4 world transform matrix
        matrix = omni.usd.get_world_transform_matrix(prim)

        # Extract 3×3 rotation matrix
        self._R = np.array([
            [matrix[0][0], matrix[0][1], matrix[0][2]],
            [matrix[1][0], matrix[1][1], matrix[1][2]],
            [matrix[2][0], matrix[2][1], matrix[2][2]]
        ])

        return self._R

    def quaternion(self, name, q=None):
        """Get quaternion orientation for the specified prim.
        
        Parameters
        ----------
        name : str
            Name of the prim to get quaternion for
        q : numpy.ndarray, optional
            Joint angles (not used in current implementation)
            
        Returns
        -------
        numpy.ndarray
            Quaternion as [w, x, y, z]
        """
        if name == "EE":
            name = self.ee_link_name
            
        prim = self._get_prim(name)

        # Get 4×4 world transform matrix and extract quaternion
        matrix = omni.usd.get_world_transform_matrix(prim)
        quat = matrix.ExtractRotationQuat()
        quat_np = np.array([quat.GetReal(), *quat.GetImaginary()])

        return quat_np

    def C(self, q=None, dq=None):
        """Get Coriolis and centrifugal forces.
        
        Note: Currently returns forces for controlled joints only.
        
        Parameters
        ----------
        q : numpy.ndarray, optional
            Joint angles
        dq : numpy.ndarray, optional
            Joint velocities
            
        Returns
        -------
        numpy.ndarray
            Coriolis and centrifugal forces for controlled joints
        """
        coriolis = self.articulation_view.get_coriolis_and_centrifugal_forces(
            joint_indices=self.dof_indices
        )
        return coriolis[self.env_idx]

    def T(self, name, q=None, x=None):
        """Get full transform matrix for the specified prim.

        Parameters
        ----------
        name : str
            Name of the prim to get transform for
        q : numpy.ndarray, optional
            Joint angles
        x : numpy.ndarray, optional
            Additional state variables
            
        Raises
        ------
        NotImplementedError
            This method is not currently implemented
        """
        raise NotImplementedError("Full transform matrix not implemented yet")

    def Tx(self, name, q=None, x=None):
        """Get position of the specified prim.

        Parameters
        ----------
        name : str
            Name of the prim to get position for
        q : numpy.ndarray, optional
            Joint angles (not used in current implementation)
        x : numpy.ndarray, optional
            Additional state variables
            
        Returns
        -------
        numpy.ndarray
            Position as [x, y, z] in meters
            
        Raises
        ------
        RuntimeError
            If prim is not found or invalid
        """
        if name == "EE":
            name = self.ee_link_name
            
        prim = self._get_prim(name)
        if not prim.IsValid():
            raise RuntimeError(f"Invalid prim: {name}")

        matrix = omni.usd.utils.get_world_transform_matrix(prim)
        position = matrix.ExtractTranslation()
        
        return np.array([position[0], position[1], position[2]], dtype=np.float64)

    def T_inv(self, name, q=None, x=None):
        """Get inverse transform matrix for the specified prim.

        Parameters
        ----------
        name : str
            Name of the prim to get inverse transform for
        q : numpy.ndarray, optional
            Joint angles
        x : numpy.ndarray, optional
            Additional state variables
            
        Raises
        ------
        NotImplementedError
            This method is not currently implemented
        """
        raise NotImplementedError("Inverse transform matrix not implemented yet")

    def _get_prim_path(self, name):
        """Find the full prim path for a given prim name.
        
        Parameters
        ----------
        name : str
            Name of the prim to find
            
        Returns
        -------
        str or None
            Full prim path if found, None otherwise
        """
        for prim in self.stage.Traverse():
            if str(prim.GetPath()).endswith(name):
                return prim.GetPath()
        return None

    def _get_prim(self, name):
        """Get USD prim object for the given name.
        
        Parameters
        ----------
        name : str
            Name of the prim to retrieve
            
        Returns
        -------
        pxr.Usd.Prim
            USD prim object
        """
        prim_path = self._get_prim_path(name)
        prim = self.stage.GetPrimAtPath(prim_path)
        return prim

    def _get_joint_positions(self):
        """Get current positions of controlled joints.
        
        Returns
        -------
        numpy.ndarray
            Joint positions for controlled joints
        """
        full_positions = self.articulation_view.get_joint_positions()[self.env_idx]
        return full_positions[self.dof_indices]

    def _get_joint_velocities(self):
        """Get current velocities of controlled joints.
        
        Returns
        -------
        numpy.ndarray
            Joint velocities for controlled joints
        """
        full_velocities = self.articulation_view.get_joint_velocities()[self.env_idx]
        return full_velocities[self.dof_indices]

    def _set_joint_positions(self, q):
        """Set positions for controlled joints.
        
        Parameters
        ----------
        q : numpy.ndarray
            Target joint positions
        """
        full_positions = self.articulation_view.get_joint_positions()[self.env_idx]
        full_positions[self.dof_indices] = q
        self.articulation_view.set_joint_positions(full_positions)

    def _set_joint_velocities(self, dq):
        """Set velocities for controlled joints.
        
        Parameters
        ----------
        dq : numpy.ndarray
            Target joint velocities
        """
        full_velocities = self.articulation_view.get_joint_velocities()[self.env_idx]
        full_velocities[self.dof_indices] = dq
        self.articulation_view.set_joint_velocities(full_velocities)