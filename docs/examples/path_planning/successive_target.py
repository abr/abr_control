"""
Example of moving between several randomly generated points where we stop at
each target (v=0)
"""
import numpy as np
import matplotlib.pyplot as plt

from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.velocity_profiles import Gaussian

n_targets = 5

Pprof = Linear()
Vprof = Gaussian(dt=0.001, acceleration=1)

path_planner = PathPlanner(
        pos_profile=Pprof,
        vel_profile=Vprof
)
start = np.zeros(3)
targets = np.random.uniform(low=-5, high=5, size=(n_targets, 3))

for ii, target in enumerate(targets):
    if ii == 0:
        start_position = start
    else:
        start_position = targets[ii-1]

    path = path_planner.generate_path(
            start_position=start_position,
            target_position=target,
            max_velocity=2
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
