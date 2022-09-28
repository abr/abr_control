"""
Example of moving between several randomly generated points where we stop at
each target (v=0)
"""
import matplotlib.pyplot as plt
import numpy as np

from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.velocity_profiles import Gaussian

n_targets = 3
dt = 0.001
np.random.seed(12)
Pprof = Linear()
Vprof = Gaussian(dt=dt, acceleration=1)

path_planner = PathPlanner(pos_profile=Pprof, vel_profile=Vprof, verbose=True)
start = np.zeros(6)
targets = np.random.uniform(low=-15, high=15, size=(n_targets, 3))
yaws = np.random.uniform(low=-3.14, high=3.14, size=n_targets)

for ii, target in enumerate(targets):
    if ii == 0:
        start_position = start[:3]
        start_orientation = start[3:]
    else:
        start_position = targets[ii - 1]
        start_orientation = [0, 0, yaws[ii - 1]]

    path = path_planner.generate_path(
        start_position=start_position,
        target_position=target,
        max_velocity=6,
        start_orientation=start_orientation,
        target_orientation=[0, 0, yaws[ii]],
        # plot=True
    )

    if ii == 0:
        position_path = path[:, :3]
        velocity_path = path[:, 3:6]
        orientation_path = path[:, 6:9]
        angvel_path = path[:, 9:]
    elif ii > 0:
        position_path = np.vstack((position_path, path[:, :3]))
        velocity_path = np.vstack((velocity_path, path[:, 3:6]))
        orientation_path = np.vstack((orientation_path, path[:, 6:9]))
        angvel_path = np.vstack((angvel_path, path[:, 9:]))

times = np.arange(0, position_path.shape[0] * dt, dt)

plt.rcParams.update({"font.size": 16})
plt.figure(figsize=(15, 6))
plt.suptitle("Reference Trajectory")
plt.subplot(221)
# plt.title("Position Path")
plt.plot(times, position_path)
plt.legend(["x", "y", "z"], loc=1)
plt.ylabel("Position [m]")
plt.xlabel("Time [sec]")

plt.subplot(222)
# plt.title("Velocity Path")
plt.plot(times, velocity_path)
plt.plot(times, np.linalg.norm(velocity_path, axis=1))
plt.legend(["dx", "dy", "dz", "norm"], loc=1)
plt.ylabel("Velocity [m/s]")
plt.xlabel("Time [sec]")

plt.subplot(223)
# plt.title("Orientation Path")
plt.plot(times, orientation_path)
plt.legend(["pitch", "roll", "yaw"], loc=1)
plt.ylabel("Orientation [rad]")
plt.xlabel("Time [sec]")

plt.subplot(224)
# plt.title("Angular Velocity Path")
plt.plot(times, angvel_path)
plt.legend(["dpitch", "droll", "dyaw"], loc=1)
plt.ylabel("Angular Velocity [rad/s]")
plt.xlabel("Time [sec]")

plt.tight_layout()
plt.show()
