from abr_control.utils import Target
import numpy as np

targets = []
# for ii in range(0,1000):
#     targets.append(np.copy(Target.random_target(
#         r=[0.4, 0.8], theta=[0.57, 6.2], phi=[2.1, 4.21])))
from abr_control.utils import DataHandler
dat = DataHandler(db_name='dewolf2018neuromorphic')
targets=dat.load(['autogen_targets'],'1lb_random_target')['autogen_targets']

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
for ii in range(0,len(targets)):
    ax.text(x[ii], y[ii], z[ii], '%i: (%.2f, %.2f, %.2f)' %(ii,x[ii], y[ii],
        z[ii]))
for angle in range(0, 360, 2):
    print('%.2f complete'%(angle/360*100), end='\r')
    ax.view_init(30, angle)
    plt.savefig('figures/gif_figs/%05d'%angle)
bashCommand = ("convert -delay 5 -loop 0 -deconstruct -quantize"
               + " transparent -layers optimize -resize 1200x2000"
               + " figures/gif_figs/*.png figures/gif_figs/targets.gif")
print('100.00% complete')
print('Creating gif...')
import subprocess
process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
output, error = process.communicate()
print('Finished')
plt.show()
# for target in targets:
#     print('target:', target)
#     plt.plot(target[0], target[1], target[2])
