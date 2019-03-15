from abr_control.controllers import path_planners
# from abr_control.utils import DataHandler, ProcessData, make_gif
# from abr_control.utils.paths import figures_dir
from abr_analyze.utils.paths import figures_dir
save_loc = figures_dir
import numpy as np
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from abr_analyze.utils import MakeGif
import os

gif = MakeGif()
gif_fig_cache = gif.prep_fig_cache()
# files = [f for f in os.listdir('%s/gif_fig_cache'%save_loc) if f.endswith(".png") ]
# for ii, f in enumerate(files):
#     if ii == 0:
#         print('Deleting old temporary figures for gif creation...')
#     os.remove(os.path.join('%s/gif_fig_cache'%save_loc, f))

path = path_planners.dmpFilter()

target_xyz = np.array([0.46324722, 0.11876897, 0.57485475])
start_xyz = np.array([0.0020, -0.0044, 1.1871])

path.generate_path_function(
        target_xyz=target_xyz,
        start_xyz=start_xyz,
        time_limit=4,
        target_vel=True)
t = np.linspace(0,4,100)
traj = []
for tt in t:
    trajectory = path.next_timestep(t=tt)
    traj.append(trajectory)
traj=np.array(traj)
plt.figure()
ax1 = plt.subplot(121, projection='3d')
ax1.view_init(30, 45)
ax2 = plt.subplot(122)
start = len(t)-1
for ii in range(start,len(t)):
    ax1.set_xlim3d(-0.35,0.35)
    ax1.set_ylim3d(-0.35,0.35)
    ax1.set_zlim3d(0.5,1.2)
    ax1.set_title('Time: %.2f'%t[ii])
    ax1.plot(traj[:ii, 0], traj[:ii, 1], traj[:ii, 2])
    ax1.scatter(target_xyz[0], target_xyz[1], target_xyz[2], label='target')
    ax1.scatter(start_xyz[0], start_xyz[1], start_xyz[2], label='start')
    ax2.plot(t[:ii], traj[:ii, 3], label='dx')
    ax2.plot(t[:ii], traj[:ii, 4], label='dy')
    ax2.plot(t[:ii], traj[:ii, 5], label='dz')
    ax2.legend()
    ax2.set_xlim(0, t[-1])
    ax2.set_ylim(-1.3,1.3)
    if start == len(t)-1:
        plt.show()
    else:
        plt.savefig('%s/%05d.png'%(gif_fig_cache,ii))
        ax1.clear()
        ax2.clear()

if start != len(t)-1:
    gif.create(fig_loc=gif_fig_cache,
                        save_loc='%s'%(figures_dir),
                        save_name='3d_filters',
                        delay=5, res=[1920,1080])


