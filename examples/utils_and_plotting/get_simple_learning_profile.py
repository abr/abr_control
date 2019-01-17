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

# NOTE: if not using hdf5 database, just load in your own data for
# input_signal
use_db = False
if use_db:
    test_group = 'testing'
    test_name = 'nengo_loihi_tuning_106_0'
    db_name = 'loihi_tuning_2019'
    loc = '/%s/%s/'%(test_group, test_name)
    dat = DataHandler(use_cache=True, db_name=db_name)
    run_data = dat.load(params=['input_signal'],
            save_location='%ssession000/run000'%(loc))
    input_signal = run_data['input_signal']
else:
    db_name = None
    test_group = None
    test_name = None
    input_signal = np.ones((1000,3))

# TODO: need to clean this up in the final form
# default in dynadapt using triangular intercepts, if using them keep
# intercepts=None, and pass in your intercepts_bounds and intercepts_mode
# if you want to manually set your intercepts you can pass them in directly
# here and intercepts_mode and intercepts_bounds will be ignored

# Define your intercepts
intercepts = None

# Define your encoders
encoders = None

plt_learning = PlotLearningProfile(test_group=test_group,
        test_name=test_name, db_name=db_name, use_cache=True,
        intercepts_bounds=[-0.7, -0.6], intercepts_mode=-0.7,
        encoders=encoders, intercepts=intercepts, #use_spherical=False,
        n_inputs=3, n_outputs=1, n_neurons=10000, n_ensembles=1)

plt_learning.plot_activity(input_signal=input_signal, time=None,
        save_num=0, plot_all_ens=False, error=None, q=None, dq=None,
        q_baseline=None, dq_baseline=None)

# a figure is created for each run, combine them into a gif
load_loc = '%s/figures/gif_fig_cache'%cache_dir
save_loc = '%s/figures/%s/learning_profile'%(cache_dir, db_name)
if not os.path.exists(save_loc):
    os.makedirs(save_loc)

make_gif.create(fig_loc=load_loc,
         save_loc=save_loc,
         save_name=test_name,
         delay=200)
