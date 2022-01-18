# This script shows an example of planning a path where the final target velocity
# is non-zero The path planner will make sure that our path ends at the target
# position at the moment it reaches its target velocity

# If we are planning successive targets with non-zero target velocities between
# target positions, we should make sure that these points sit along the same line
# if possible to reduce any jerk in the velocity path. The path planner plans a
# path directly from start to target position, and only warps it to follow the
# path profile along that straight line. If we are starting at a non-zero velocity
# that points along a different direction, which would happen if our previously
# planned path ended in a non-zero target velocity, then we need to abruptly
# adjust those components so that we move in the desired direction along our
# path. The norm of the velocity will still remain smooth, but the individual
# component may jump.

import matplotlib.pyplot as plt
import numpy as np

from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.velocity_profiles import Gaussian

Pprof = Linear()
Vprof = Gaussian(dt=0.001, acceleration=1)

path_planner = PathPlanner(
    pos_profile=Pprof,
    vel_profile=Vprof
)
start = np.zeros(3)
targets = np.array([
    [2, 3, 4],
    [4, 6, 8]]
)

target_velocities = [1, 0.5]

for ii, target in enumerate(targets):
    if ii == 0:
        start_position = start
        start_velocity = 0
    else:
        start_position = targets[ii-1]
        start_velocity = target_velocities[ii-1]

    path = path_planner.generate_path(
        start_position=start_position,
        target_position=target,
        max_velocity=2,
        start_velocity=start_velocity,
        target_velocity=target_velocities[ii],
    )

    if ii == 0:
        position_path = path[:, :3]
        velocity_path = path[:, 3:6]
    elif ii > 0:
        position_path = np.vstack((position_path, path[:, :3]))
        velocity_path = np.vstack((velocity_path, path[:, 3:6]))

plt.figure()
plt.subplot(211)
plt.title('Position Path')
plt.plot(position_path)

plt.subplot(212)
plt.title('Velocity Path')
plt.plot(velocity_path)
plt.plot(np.linalg.norm(velocity_path, axis=1))
plt.legend(['dx', 'dy', 'dz', 'norm'])
plt.show()
