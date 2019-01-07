from abr_control.utils import DataHandler, ProcessData
import numpy as np
import matplotlib
#matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

def convert_to_spherical(input_signal):
    x = input_signal
    pi = np.pi
    spherical = []

    def scale(input_signal, factor):
        #TODO: does it make more sense to pass in the range and have the script
        # handle the division, so we go from 0-factor instead of 0-2*factor?
        """
        Takes inputs in the range of -1 to 1 and scales them to the range of
        0-2*factor

        ex: if factor == pi the inputs will be in the range of 0-2pi
        """
        signal = np.copy(input_signal)
        for ii, dim in enumerate(input_signal):
            signal[ii] = dim * factor + factor
        return signal

    def sin_product(input_signal, count):
        """
        Handles the sin terms in the conversion to spherical coordinates where
        we multiple by sin(x_i) n-1 times
        """
        tmp = 1
        for jj in range(0, count):
            tmp *= np.sin(input_signal[jj])
        return tmp

    # nth input scaled to 0-2pi range, remainder from 0-pi
    # cycle through each input
    x_rad = scale(input_signal=x, factor=pi/2)
    x_2rad = scale(input_signal=x, factor=pi)

    for ss in range(0, len(x)):
        sphr = sin_product(input_signal=x_rad, count=ss)
        sphr*= np.cos(x_rad[ss])
        spherical.append(sphr)
    spherical.append(sin_product(input_signal=x_2rad, count=len(x)))
    return(spherical)

dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')
proc = ProcessData()
#loc = '/1lb_random_target/nengo_cpu_46_59/session000'
loc = '/friction_post_tuning/nengo_loihi_friction_10_0/session000'
n_runs = 50
n_points = 400
for ii in range(0,n_runs):
    data = dat.load(params=['q', 'dq', 'time'], save_location='%s/run%03d'%(loc, ii))
    q = data['q']
    dq = data['dq']
    time = data['time']
    # interpolate data for even sampling
    q = proc.interpolate_data(data=q, time_intervals=time, n_points=n_points)
    dq = proc.interpolate_data(data=dq, time_intervals=time, n_points=n_points)
    if ii == 0:
        qs = q
        dqs = dq
    else:
        qs = np.vstack((qs, q))
        dqs = np.vstack((dqs, dq))

qs = qs.T
dqs = dqs.T
qs[0] = (qs[0] + np.pi) % (2*np.pi)
qs[4] = (qs[4] + np.pi) % (2*np.pi)

qs = [qs[0], qs[1], qs[2], qs[3], qs[4]]
dqs = [dqs[0], dqs[1], dqs[2], dqs[3], dqs[4]]

MEANS = {  # expected mean of joint angles / velocities
    # shift from 0-2pi to -pi to pi
    'q': np.array([3.20, 2.14, 1.52, 4.68, 3.00, 3.00]),
    'dq': np.array([0.002, -0.117, -0.200, 0.002, -0.021, 0.002]),
    }
SCALES = {  # expected variance of joint angles / velocities
    'q': np.array([0.2, 1.14, 1.06, 1.0, 2.8, 0.01]),
    'dq': np.array([0.06, 0.45, 0.7, 0.25, 0.4, 0.01]),
    }
for ii in range(0, 5):
    qs[ii] = (qs[ii] - MEANS['q'][ii]) / SCALES['q'][ii]
    dqs[ii] = (dqs[ii] - MEANS['dq'][ii]) / SCALES['dq'][ii]
# import pdb; pdb.set_trace()
#

inputs = np.vstack((qs, dqs))
inputs = convert_to_spherical(input_signal = inputs)
qs = inputs[0:5]
dqs = inputs[5:11]
print('spherical: ', np.array(inputs).shape)


colors = ['r', 'g', 'b', 'm', 'y', 'k', 'tab:grey']

print('q: ', np.array(qs).shape)
print('dq: ', np.array(dqs).shape)
fig = plt.figure()
ax = fig.add_subplot(2,1,1)
plt.grid()
handles = []
for ii in range(0, len(qs)):
    data = qs[ii]
    violin_parts = ax.violinplot(
        data, [ii],
        showmeans=True, showextrema=False, points=1000)
    violin_parts['cmeans'].set_edgecolor('black')#colors[ii])
    violin_parts['cmeans'].set_linewidth(3)
    violin_parts['bodies'][0].set_facecolor(colors[ii])
    violin_parts['bodies'][0].set_alpha(.8)
plt.title('q')

ax = fig.add_subplot(2,1,2)
plt.grid()
handles = []
for ii in range(0, len(dqs)):
    data = dqs[ii]
    violin_parts = ax.violinplot(
        data, [ii],
        showmeans=True, showextrema=False, points=1000)
    violin_parts['cmeans'].set_edgecolor('black')#colors[ii])
    violin_parts['cmeans'].set_linewidth(3)
    violin_parts['bodies'][0].set_facecolor(colors[ii])
    violin_parts['bodies'][0].set_alpha(.8)
plt.title('dq')
ax.set_ylim(-1,1)
plt.savefig('q-dq_means_and_scales.pdf')
plt.show()
# [q_mean, q_lower_bound, q_upper_bound] = proc.get_mean_and_ci(raw_data=qs,
#         n_runs=6)
# [dq_mean, dq_lower_bound, dq_upper_bound] = proc.get_mean_and_ci(raw_data=dqs,
#         n_runs=6)
#
# plt.figure()
# plt.subplot(2,1,1)
# plt.title('q')
# plt.plot(q_mean, label='q mean')
# plt.plot(q_upper_bound, label='q upper')
# plt.plot(q_lower_bound, label='q upper')
# plt.legend()
# plt.subplot(2,1,2)
# plt.title('dq')
# plt.plot(dq_mean, label='dq mean')
# plt.plot(dq_upper_bound, label='dq upper')
# plt.plot(dq_lower_bound, label='dq upper')
# plt.legend()
# plt.savefig('q-dq_means_and_scales.pdf')
# plt.show()
