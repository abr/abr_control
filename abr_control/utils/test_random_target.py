from abr_control.utils import Target
import numpy as np

targets = []
for ii in range(0,1000):
    targets.append(np.copy(Target.random_target(
        r=[0.4, 0.8], theta=[0.57, 6.2], phi=[2.1, 4.21])))

import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

targets = np.array(targets)
x = targets[:,0]
y = targets[:,1]
z = targets[:,2]
# phi = targets[:,3]
# theta = targets[:,4]
# print(x.shape)
# print(theta.shape)
# print(phi.shape)
# for ii in range(0, len(x)):
#     if x[ii] < 0:
#         print('-----------')
#         print('x: ', x[ii], '\ntheta: ', theta[ii], '\nphi: ', phi[ii])
ax.scatter(x,y,z,c='r',marker='o')
plt.show()
# for target in targets:
#     print('target:', target)
#     plt.plot(target[0], target[1], target[2])
