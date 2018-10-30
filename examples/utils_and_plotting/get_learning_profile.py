"""
Runs a simulation using the data recorded from a run of learning to get
information about the neural network
Plots will be created for each reach, or the inputs can be stacked to get
activity over the whole reaching space

The intercepts bounds and mode can be altered to see how it would affect the
network provided the same input. This helps tune your network without having to
rerun tests

Plots
1. rasterplot showing spikes for each neuron over time
2. proportion of time active, the number of neurons active for what proportion
   of run time
3. proportion of neurons that are active over time
"""
from abr_control.utils import DataHandler, PlotLearningProfile, make_gif
from abr_control.utils.paths import cache_dir
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

test_group = 'weighted_reach_post_tuning'
test_name = 'nengo_cpu_weight_12_9'
db_name = 'dewolf2018neuromorphic'
loc = '/%s/%s/'%(test_group, test_name)
baseline_loc = '/weighted_reach_post_tuning/pd_no_weight_4/'
use_cache = True
n_runs = 10
use_spherical = False

plt_learning = PlotLearningProfile(test_group=test_group,
        test_name=test_name, db_name=db_name, use_cache=use_cache,
        intercepts_bounds=[-0.5, -0.4], intercepts_mode=-0.5,
        use_spherical=use_spherical)

dat = DataHandler(use_cache=use_cache, db_name=db_name)
# pass input signals for target reaches individually, creates gif of
# activity for each individual reach
session = 0
for run in range(0, n_runs):
    # load data from the current run
    run_data = dat.load(params=['input_signal', 'time', 'error', 'q', 'dq'],
            save_location='%ssession%03d/run%03d'
            %(loc, session, run))
    training_data = dat.load(params=['adapt_input'],
            save_location='%s/parameters/training' %loc)
    adapt_input = training_data['adapt_input']
    input_signal = run_data['input_signal']
    time = run_data['time']
    # TODO: delete the following lines after debugging up to and not including the
    # plot_activity function call
    error = run_data['error']
    q = run_data['q']
    q = np.array([q[:,1], q[:,2]])
    dq = run_data['dq']
    dq = np.array([dq[:,1], dq[:,2]])
    pd_run_data = dat.load(params=['q', 'dq'],
            save_location='%ssession%03d/run%03d'
            %(baseline_loc, session, run))
    q_baseline = pd_run_data['q']
    q_baseline = np.array([q_baseline[:,1], q_baseline[:,2]])
    dq_baseline = pd_run_data['dq']
    dq_baseline = np.array([dq_baseline[:,1], dq_baseline[:,2]])
    print('q: ', q.shape)
    print('dq: ', dq.shape)
    print('q_baseline: ', q_baseline.shape)
    print('dq_baseline: ', dq_baseline.shape)
    # run the simulation to obtain neural activity
    plt_learning.plot_activity(input_signal=input_signal, time=time,
            save_num=run, plot_all_ens=False, error=error, q=q, dq=dq,
            q_baseline=q_baseline, dq_baseline=dq_baseline)

# stack input signal to show activity over the enitre reaching space
# adapt_input = []
# input_signal = []
# time = []
# for run in range(0, n_runs):
#     run_data = dat.load(params=['input_signal', 'time', 'error', 'q', 'dq'],
#             save_location='%ssession%03d/run%03d'
#             %(loc, session, run))
#     training_data = dat.load(params=['adapt_input'],
#             save_location='%s/parameters/training' %loc)
#     adapt_input.append(training_data['adapt_input'])
#     input_signal.append(run_data['input_signal'])
#     time.append(run_data['time'])
#     error = run_data['error']
#     q = run_data['q']
#     dq = run_data['dq']
#
# input_signal = np.vstack(input_signal)
# time = np.hstack(time)
# plt_learning.plot_activity(input_signal=input_signal, time=time,
#         error=error, save_num=run, q=q, dq=dq, input_joints=adapt_input)


# a figure is created for each run, combine them into a gif
load_loc = '%s/figures/gif_fig_cache'%cache_dir
save_loc = '%s/figures/%s/learning_profile'%(cache_dir, db_name)
if not os.path.exists(save_loc):
    os.makedirs(save_loc)

make_gif.create(fig_loc=load_loc,
         save_loc=save_loc,
         save_name=test_name,
         delay=200)
