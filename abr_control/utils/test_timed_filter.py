from abr_control.controllers import path_planners
import timeit
import time
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np

path = path_planners.SecondOrder(robot_config=None,
        n_timesteps=3000, w=1e4, zeta=3, threshold=0.0)
start = np.array([0.2, 0.3, 0.22, 0, 0, 0])
target = np.array([-0.33, 0.76, 0.77])
path.generate_path_function(state=start, target_pos=target, runtime=4)
times = 0
limit = 4
paths = []
while times < limit:
    now = timeit.default_timer()
    time.sleep(0.1)
    paths.append(path.next_timestep(times))
    times += timeit.default_timer()-now

final = path.next_timestep(4)
paths = np.array(paths)
plt.figure()
ax = plt.subplot(111, projection='3d')
ax.plot(paths[:,0], paths[:,1], paths[:,2])
ax.scatter(target[0], target[1], target[2], c='r')
ax.scatter(start[0], start[1], start[2], c='g')
ax.scatter(final[0], final[1], final[2], c='k')
plt.title(times)
plt.show()
