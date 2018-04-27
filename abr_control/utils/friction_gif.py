import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib import gridspec
import numpy as np

friction = np.squeeze(np.load('friction0.npz')['friction'])
kf = np.squeeze(np.load('kfriction0.npz')['kfriction'])
print('shape: ', friction.shape)
print('kfriction: ', kf)
x_times = np.ones(len(friction))
for ii in range(0, len(friction)):
    x_times[ii] *= 20*ii/(len(friction))

print(x_times.shape)

for ii in range(0, len(friction), 10):
    fig = plt.figure(figsize=(16,2))
    gs = gridspec.GridSpec(1,2, width_ratios=[1,3])
    ax = fig.add_subplot(gs[1])
    ax.set_title('Simulated Joint Friction')
    ax.set_ylabel('Friction [Nm]')
    ax.set_xlabel('Run Time [sec]')
    ax.plot(x_times[0:ii],friction[0:ii, 1], label='Joint 2')
    ax.plot(x_times[0:ii],friction[0:ii, 0], label='Joint 1')
    ax.set_ylim(-4,4)
    ax.set_xlim(0,20)
    ax.legend()
    ax2 = fig.add_subplot(gs[0])
    ax2.set_ylim(0,3)
    ax2.set_xlim(0,25)
    ax2.text(1,2, 'Year Simulated: 0', fontsize=20)
    ax2.xaxis.set_visible(False)
    ax2.yaxis.set_visible(False)
    # ax2.text(1, 2.5, "Joint 1 Friction: %f Nm"% friction[ii,0])
    # ax2.text(1, 2, "Joint 2 Friction: %f Nm"% friction[ii,1])
    # ax2.text(1, 1, 'Simulated Friction Time:\n %f years'%(3*ii/len(friction)))
    # ax2.text(1, 0.5, 'Actual Run Time:\n %f minutes'%((50*0.333)*ii/len(friction)))
    plt.tight_layout()
    plt.savefig('friction_images/%05d.'%ii)
    plt.close()
    if ii%50 == 0:
        print('%f%% complete'%(ii/len(friction)*100), end='\r')

bashCommand = ("convert -delay 5 -loop 0 -deconstruct -quantize"
               + " transparent -layers optimize -resize 1200x2000"
               + " friction_images/*.png animation.gif")
print('100.000000% complete')
print('Creating gif...')
import subprocess
process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
output, error = process.communicate()
print('Finished')
