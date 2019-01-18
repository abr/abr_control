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
def sample_encoders(input_signal, n_neurons=1000):
    iterations = int(1.1*n_neurons)
    ii = 0
    same_count = 0
    prev_index = 0
    thresh = .008
    while (input_signal.shape[0] > n_neurons):
    #for ii in range(iterations):
        if ii%500 == 0:
            print(input_signal.shape)
            print('thresh: ', thresh)
        # choose a random set of indices
        n_indices = input_signal.shape[0]
        # make sure we're dealing with an even number
        n_indices -= 0 if ((n_indices % 2) == 0) else 1
        n_half = int(n_indices / 2)

        randomized_indices = np.random.permutation(range(n_indices))
        a = randomized_indices[:n_half]
        b = randomized_indices[n_half:]

        data1 = input_signal[a]
        data2 = input_signal[b]

        distances = np.linalg.norm(data1 - data2, axis=1)

        under_thresh = distances > thresh

        input_signal = np.vstack([data1, data2[under_thresh]])
        ii += 1
        if prev_index == n_indices:
            same_count += 1
        else:
            same_count = 0

        if same_count == 500:
            same_count = 0
            if thresh > 0.001:
                thresh += 0.001
            elif thresh > 0.0001:
                thresh -= 0.0001
            else:
                print(distances)
                import time
                time.sleep(100)

            print('All values are within threshold, but not at target size.')
            print('Dropping threshold to %.4f' %thresh)
        prev_index = n_indices

    encoders = convert_to_spherical(input_signal)
    return encoders

# convert to 3D
def convert_to_spherical(input_signal):
    """ converts an input signal of shape time x N_joints and converts to spherical
    """
    print('IN: ', input_signal.shape)
    x = input_signal.T
    pi = np.pi
    spherical = []

    def scale(input_signal):
        #TODO: does it make more sense to pass in the range and have the script
        # handle the division, so we go from 0-factor instead of 0-2*factor?
        """
        Takes inputs in the range of -1 to 1 and scales them to the range of
        0-2*factor

        ex: if factor == pi the inputs will be in the range of 0-2pi
        """
        signal = np.copy(input_signal)
        factor = pi
        for ii, dim in enumerate(input_signal):
            if ii == len(input_signal)-1:
                factor = 2*pi
            signal[ii] = dim * factor# + factor
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
    x_rad = scale(input_signal=x)

    for ss in range(0, len(x)):
        sphr = sin_product(input_signal=x_rad, count=ss)
        sphr*= np.cos(x_rad[ss])
        spherical.append(sphr)
    spherical.append(sin_product(input_signal=x_rad, count=len(x)))
    spherical = np.array(spherical).T
    print('OUT: ', np.array(spherical).shape)
    return(spherical)


# NOTE: if not using hdf5 database, just load in your own data for
# input_signal
use_db = True
if use_db:
    test_group = 'testing'
    test_name = 'nengo_loihi_tuning_106_0'
    db_name = 'loihi_tuning_2019'
    loc = '/%s/%s/'%(test_group, test_name)
    dat = DataHandler(use_cache=True, db_name=db_name)
    run_data = dat.load(params=['input_signal', 'input_signal_non_spherical'],
            save_location='%ssession000/run000'%(loc))
    input_signal_spherical = run_data['input_signal']
    input_signal_non_spherical = run_data['input_signal_non_spherical']
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

n_neurons = 1000

# Define your intercepts
intercepts = None

# Define your encoders
encoders = sample_encoders(input_signal=input_signal_non_spherical,
        n_neurons=n_neurons)

plt_learning = PlotLearningProfile(test_group=test_group,
        test_name=test_name, db_name=db_name, use_cache=True,
        intercepts_bounds=[-0.5, -0.1], intercepts_mode=-0.4,
        encoders=encoders, intercepts=intercepts, #use_spherical=False,
        n_inputs=3, n_outputs=1, n_neurons=n_neurons, n_ensembles=1)

plt_learning.plot_activity(input_signal=input_signal_spherical, time=None,
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
