import numpy as np
from abr_control.utils import download_meshes, transformations
from isaacsim import SimulationApp
from .interface import Interface


class IsaacSim(Interface):
    """An interface for IsaacSim.

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    dt : float, optional (Default: 0.001)
        simulation time step in seconds

    """
    def __init__(self, robot_config, dt=0.001):

        super().__init__(robot_config)

        self.dt = dt  # time step
        self.count = 0  # keep track of how many times send forces is called

        self.q = np.zeros(self.robot_config.N_JOINTS)  # joint angles
        self.dq = np.zeros(self.robot_config.N_JOINTS)  # joint_velocities

        self.robot_model = None
        self.urdf_path = None
        self.import_config = None
        self.world = None

        
        #self.misc_handles = {}  # for tracking miscellaneous object handles

        '''
        # joint target velocities, as part of the torque limiting control
        # these need to be super high so that the joints are always moving
        # at the maximum allowed torque
        self.joint_target_velocities = np.ones(robot_config.N_JOINTS) * 10000.0
        '''
        

    def connect(self, load_scene=True):

        
        if load_scene:
            """All initial setup."""
            self.simulation_app = SimulationApp({"headless": False}) 
            from isaacsim.core.api import World # type: ignore
            from isaacsim.core.api.objects import DynamicCuboid # type: ignore
            from isaacsim.core.utils.extensions import get_extension_path_from_name # type: ignore
            from isaacsim.asset.importer.urdf import _urdf # type: ignore
            import omni.kit.commands # type: ignore
            import omni.usd # type: ignore
            import omni
            # Create a world
            self.world = World(physics_dt=self.dt,rendering_dt=self.dt)
            self.world.scene.add_default_ground_plane()


            # Acquire the URDF extension interface for parsing and importing URDF files
            urdf_interface = _urdf.acquire_urdf_interface()

            # Configure the settings for importing the URDF file
            self.import_config = _urdf.ImportConfig()
            self.import_config.convex_decomp = False  # Disable convex decomposition for simplicity
            self.import_config.fix_base = True       # Fix the base of the robot to the ground
            self.import_config.make_default_prim = True  # Make the robot the default prim in the scene
            self.import_config.self_collision = False  # Disable self-collision for performance
            self.import_config.distance_scale = 1     # Set distance scale for the robot
            self.import_config.density = 0.0          # Set density to 0 (use default values)


            # Retrieve the path of the URDF file from the extension
            extension_path = get_extension_path_from_name("isaacsim.asset.importer.urdf")
            root_path = extension_path + "/data/urdf/robots/ur10/urdf"
            #root_path = extension_path + "/data/urdf/robots/ur5/urdf"
            file_name = "ur10.urdf"
            #file_name = "ur5.urdf"

            # Parse the robot's URDF file to generate a robot model
            result, self.robot_model = omni.kit.commands.execute(
                "URDFParseFile",
                urdf_path="{}/{}".format(root_path, file_name),
                import_config=self.import_config
                )
            # Resetting the world needs to be called before querying anything related to an articulation specifically.
            # Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
            self.world.reset()


        else:
            self.world = SimulationApp.getWorld()
            

        # Get the articulation
        from omni.isaac.core.articulations import Articulation, ArticulationSubset # type: ignore
        import omni.isaac.core.utils.stage as stage_utils # type: ignore

        # Import the robot onto the current stage and retrieve its prim path
        result, prim_path_ur = omni.kit.commands.execute(
            "URDFImportRobot",
            urdf_robot=self.robot_model,
            import_config=self.import_config,
            )
        
        # necessary so self.q and self.dq are accessible
        self.world.initialize_physics()

        # Load robot
        self.articulation = Articulation(prim_path=prim_path_ur, name="ur10")
        self.articulation.initialize()
        

        print("Started Isaacsim as stand alone app...")





    def disconnect(self):
        """Any socket closing etc that must be done to properly shut down"""
        self.simulation_app.close() # close Isaac Sim
        print("IsaacSim connection closed...")



    def send_forces(self, u):
        """Applies the set of torques u to the arm. If interfacing to
        a simulation, also moves dynamics forward one time step.

        u : np.array
            An array of joint torques [Nm]
        """
        # Apply some torque
        self.articulation.set_joint_efforts(u)

         # move simulation ahead one time step
        self.world.step(render=True) # execute one physics step and one rendering step



    def send_target_angles(self, q):
        """Moves the arm to the specified joint angles

        q : numpy.array
            the target joint angles [radians]
        """
        # Set the target angles for the joints
        from isaacsim.core.utils.types import ArticulationAction # type: ignore
        self.articulation.get_articulation_controller().apply_action(ArticulationAction(q))

        # move simulation ahead one time step
        self.world.step(render=True) # execute one physics step and one rendering step


    def get_feedback(self):
        """Return a dictionary of information needed by the controller.

        Returns the joint angles and joint velocities in [rad] and [rad/sec],
        respectively
        """
        # Get the joint angles and velocities
        self.q = self.articulation.get_joint_positions()
        self.dq = self.articulation.get_joint_velocities()
        return {"q": self.q, "dq": self.dq}


    def get_xyz(self, name):
                """Returns the xyz position of the specified object

                name : string
                    name of the object you want the xyz position of
                """
                #TODO check if we need misc handles for this
                obj = self.world.scene.get_object(name) 
                object_position, object_orientation = obj.get_world_pose()

                return object_position