from abr_control.utils import DataHandler, ProcessData
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')
proc = ProcessData()
loc = '/1lb_random_target/nengo_cpu_46_59/session000'
n_runs = 10
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
colors = ['r', 'g', 'b', 'm', 'y', 'k']

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
