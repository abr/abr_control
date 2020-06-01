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

# from abr_control.arms import twojoint as arm
from abr_control.interfaces.pygame import PyGame
from abr_control.controllers import OSC, Damping, path_planners


# if set to True, the simulation will plan the path to
# last for 1 second of real-time
use_wall_clock = True

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
params = {"w": 1e4, "zeta": 2}
if use_wall_clock:
    run_time = 1  # wall clock time to run each trajectory for
    time_elapsed = np.copy(run_time)
    count = 0
else:
    params["n_timesteps"] = 250  # time steps each trajectory lasts
    count = np.copy(params["n_timesteps"])
    time_elapsed = 0.0
path_planner = path_planners.SecondOrderFilter(**params)

# create our interface
interface = PyGame(robot_config, arm_sim, dt=0.001)
interface.connect()

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
            update_target = count == params["n_timesteps"]

        if update_target:
            count = 0
            time_elapsed = 0.0
            target_xyz = np.array(
                [np.random.random() * 2 - 1, np.random.random() * 2 + 1, 0]
            )
            # update the position of the target
            interface.set_target(target_xyz)

            J = robot_config.J("EE", feedback["q"])
            pos_path, vel_path = path_planner.generate_path(
                position=hand_xyz,
                velocity=np.dot(J, feedback["dq"])[:3],
                target_position=target_xyz,
                plot=False,
            )
            if use_wall_clock:
                pos_path = path_planner.convert_to_time(
                    path=pos_path, time_length=run_time
                )
                vel_path = path_planner.convert_to_time(
                    path=vel_path, time_length=run_time
                )

        # get next target along trajectory
        if use_wall_clock:
            target = [function(time_elapsed) for function in pos_path]
            target_velocity = [function(time_elapsed) for function in vel_path]
        else:
            target, target_velocity = path_planner.next()

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
