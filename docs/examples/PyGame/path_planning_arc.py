"""
Running the operational space control with a first order path planner
using the PyGame display. The path planner system will generate a
trajectory for the controller to follow, moving the end-effector in a
in an arc to the target. Two circles are drawn, one with a radius of
origin to EE, and the other from origin to target. The path is taken
from these circles by linearly weighting their paths starting with
full weighting at the EE arc to full weighting at the target arc by
the end of the path.

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
from abr_control.controllers.path_planners.position_profiles import Ellipse
from abr_control.controllers.path_planners.velocity_profiles import Linear

# from abr_control.arms import twojoint as arm
from abr_control.interfaces.pygame import PyGame

np.random.seed(0)
dt = 0.001
# if set to True, the simulation will plan the path to
# last for 1 second of real-time
use_wall_clock = True

# initialize our robot config
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
count = 0
if use_wall_clock:
    run_time = 5  # wall clock time to run each trajectory for
    time_elapsed = np.copy(run_time)
else:
    time_elapsed = 0.0
path_planner = PathPlanner(
        pos_profile=Ellipse(horz_stretch=0.5),
        vel_profile=Linear(dt=dt, acceleration=1)
        )

# create our interface
interface = PyGame(robot_config, arm_sim, dt=dt)
interface.connect()
first_pass = True

try:
    print("\nSimulation starting...")
    print("Click to move the target.\n")

    while 1:
        start = timeit.default_timer()
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx("EE", feedback["q"])

        if use_wall_clock:
            # either update target every 1s
            update_target = time_elapsed >= run_time
        else:
            # or update target when trajectory is done
            update_target = False
            if count == path_planner.n_timesteps:
                update_target = True
            elif first_pass:
                update_target =  True
                first_pass = False

        if update_target:
            count = 0
            time_elapsed = 0.0
            target_xyz = np.array(
                [np.random.random() * 2 - 1, np.random.random() * 2 + 1, 0]
            )
            # update the position of the target
            interface.set_target(target_xyz)

            generated_path = path_planner.generate_path(
                start_position=hand_xyz,
                target_position=target_xyz,
                max_velocity=1,
                plot=False
            )
            if use_wall_clock:
                pos_path = path_planner.convert_to_time(
                        path=generated_path[:, :3],
                        time_length=path_planner.time_to_converge
                )
                vel_path = path_planner.convert_to_time(
                        path=generated_path[:, 3:6],
                        time_length=path_planner.time_to_converge
                )

        # get next target along trajectory
        if use_wall_clock:
            target = [function(min(path_planner.time_to_converge, time_elapsed))
                    for function in pos_path
            ]
            target_velocity = [
                    function(min(path_planner.time_to_converge, time_elapsed))
                    for function in vel_path
            ]
        else:
            next_target = path_planner.next()
            target = next_target[:3]
            target_velocity = next_target[3:]

        # generate an operational space control signal
        u = ctrlr.generate(
            q=feedback["q"],
            dq=feedback["dq"],
            target=np.hstack([target, np.zeros(3)]),
            target_velocity=np.hstack([target_velocity, np.zeros(3)]),
        )

        # apply the control signal, step the sim forward
        interface.send_forces(u, update_display=(count % 20 == 0))

        count += 1
        time_elapsed += timeit.default_timer() - start

finally:
    # stop and reset the simulation
    interface.disconnect()

    print("Simulation terminated...")
