import numpy as np
from abr_control.arms import ur5 as arm
from abr_control.interfaces.nv_isaacsim import IsaacSim

class FlyingCube:

    def __init__(self, dt):

        # Initialize our robot config
        robot_config = arm.Config()

        # Create our interface
        self.interface = IsaacSim(robot_config, dt=dt)
        self.interface.connect()
       
        from omni.isaac.core import World # type: ignore
        from omni.isaac.core.objects import DynamicCuboid # type: ignore
        from omni.isaac.dynamic_control import _dynamic_control # type: ignore

       
        fancy_cube = self.interface.world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="fancy_cube",
                position=np.array([0, 0, 1.0]),
                scale=np.array([0.1015, 0.1015, 0.1015]),
                color=np.array([0, 0, 1.0]),
            )
        )

        # Resetting the world needs to be called before querying anything related to an articulation specifically.
        # It's recommended to always do a reset after adding your assets, for physics handles to be propagated properly.
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self.cube_handle = self._dc.get_rigid_body("/World/random_cube")
        self._cube = self.interface.world.scene.get_object("fancy_cube")

        # Add velocity as an attribute of the world so that it is available at callback time
        self.input_velocity = np.zeros(3)
        
        self.interface.world.add_physics_callback("send_actions", self.send_actions)
        self.interface.world.add_physics_callback("make_state", self.make_state)


    def send_actions(self, dt):
        self._dc.set_rigid_body_linear_velocity(self.cube_handle, self.input_velocity)
        
    def make_state(self, dt):
        position, orientation = self._cube.get_world_pose()
        linear_velocity = self._cube.get_linear_velocity()
        return {
            'position': np.array([[0., 0., 1.]]),
            'velocity': np.array([[0., 0., 0.]]),
            'orientation': np.array([[0., 0., 0., 0.]])
        }

    def step(self, velocity):
        # Update the input velocity
        self.input_velocity = velocity
        self.interface.world.step()

        # Get the cube's current state
        position, orientation = self._cube.get_world_pose()
        linear_velocity = self._cube.get_linear_velocity()
        return np.concatenate([position, linear_velocity, orientation], axis=0)

# Example usage
if __name__ == "__main__":
    dt = 0.01
    flying_cube = FlyingCube(dt)

    flying_cube.interface.send_target_angles(np.array([1.0, 0.0,2.0,3.0,0.0,0.0]))
    feedback = flying_cube.interface.get_feedback()
    print ("q : " + str(feedback["q"]))
    print ("dq : " + str(feedback["dq"]))

    # Example: Apply sinusoidal velocity to the cube
    for t in np.arange(0, 10, dt):
        velocity = np.array([np.sin(t), np.cos(t), 0.])
        state = flying_cube.step(velocity)
        #print(f"Time: {t:.2f}, State: {state}")