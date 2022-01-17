"""
Running the operational space control with a second order path planner
using the PyGame display. The path planning system will generate
a trajectory for the controller to follow, moving the end-effector
smoothly to the target.

There are two ways to run the path planner:
    1) Trajectories last for a specified number of time steps
    2) Trajectories last for a specified length of wall clock time
To use 1, set use_wall_clock = False, to use 2, set use_wall_clock = True.
There is largely no benefit to using wall clock time in simulation, but for
implementation on real robots it can be a very helpful function.
"""
import timeit

import numpy as np

from abr_control.arms import threejoint as arm
from abr_control.controllers import OSC, Damping
from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.velocity_profiles import Gaussian
from abr_control.interfaces.pygame import PyGame

dt = 0.001
# if set to True, the simulation will plan the path to
# last for 1 second of real-time
use_wall_clock = False

# initialize our robot config for the ur5
robot_config = arm.Config()
# create our arm simulation
arm_sim = arm.ArmSim(robot_config)

# damp the movements of the arm
damping = Damping(robot_config, kv=10)
# create an operational space controller
ctrlr = OSC(
    robot_config,
    kp=200,
    null_controllers=[damping],
    # control (gamma) out of [x, y, z, alpha, beta, gamma]
    ctrlr_dof=[True, True, False, False, False, False],
)

# create our path planner
params = {}
if use_wall_clock:
    run_time = 5  # wall clock time to run each trajectory for
    time_elapsed = 0.0
    count = 0
else:
    params["error_scale"] = 50
    params["n_timesteps"] = 500  # time steps each trajectory lasts
    count = np.copy(params["n_timesteps"])
    time_elapsed = 0.0
first_pass = True

path_planner = PathPlanner(
        pos_profile=Linear(),
        vel_profile=Gaussian(dt=dt, acceleration=2)
)

# create our interface
interface = PyGame(robot_config, arm_sim, dt=dt)
interface.connect()

try:
    print("\nSimulation starting...")
    print("Click to move the target.\n")

    while 1:
        start = timeit.default_timer()
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx("EE", feedback["q"])

        if first_pass:
            update_target = True
            first_pass = False
        elif use_wall_clock:
            # either update target every 1s
            update_target = time_elapsed >= path_planner.time_to_converge
        else:
            # or update target when trajectory is done
            update_target = count == path_planner.n_timesteps

        if update_target:
            count = 0
            time_elapsed = 0.0
            target_xyz = np.array(
                [np.random.random() * 2 - 1, np.random.random() * 2 + 1, 0]
            )
            # update the position of the target
            interface.set_target(target_xyz)

            generated_path = path_planner.generate_path(
                start_position=hand_xyz, target_position=target_xyz, max_velocity=3, plot=False
            )
            pos_path = generated_path[:, :3]
            vel_path = generated_path[:, 3:6]

            if use_wall_clock:
                pos_path = path_planner.convert_to_time(
                    pregenerated_path=pos_path, time_limit=path_planner.time_to_converge
                )
                vel_path = path_planner.convert_to_time(
                    pregenerated_path=vel_path, time_limit=path_planner.time_to_converge
                )

        # get next target along trajectory
        if use_wall_clock:
            target = [function(time_elapsed) for function in pos_path]
            target_velocity = [function(time_elapsed) for function in vel_path]
        else:
            next_target = path_planner.next()
            target = next_target[:3]
            target_velocity = next_target[3:]

        # generate an operational space control signal
        u = ctrlr.generate(
            q=feedback["q"],
            dq=feedback["dq"],
            target=np.hstack((target, np.zeros(3))),
            target_velocity=np.hstack((target_velocity, np.zeros(3))),
        )

        # apply the control signal, step the sim forward
        interface.send_forces(u, update_display=(count % 20 == 0))

        count += 1
        time_elapsed += timeit.default_timer() - start

finally:
    # stop and reset the simulation
    interface.disconnect()

    print("Simulation terminated...")
