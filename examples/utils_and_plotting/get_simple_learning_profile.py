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
from abr_control.controllers import signals
from abr_control.utils.paths import cache_dir
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

def generate_encoders(input_signal=None, n_neurons=1000, thresh=0.008,
        use_spherical=True, run=0):
    """
    Accepts inputs signal in the shape of time X dim and outputs encoders
    for the specified number of neurons by sampling from the input

    *NOTE* the input must be passed in prior to spherical conversion, if
    any is being done
    """
    #TODO: scale thresh based on dimensionality of input, 0.008 for 2DOF
    # and 10k neurons, 10DOF 10k use 0.08, for 10DOF 100 went up to 0.708 by end
    # 0.3 works well for 1000
    # first run so we need to generate encoders for the sessions
    debug = False
    if debug:
        print('\n\n\nDEBUG LOAD ENCODERS\n\n\n')
        encoders = np.load('encoders-backup.npz')['encoders']
    elif run == 0:
        print('First run of session, generating encoders...')
        if input_signal is None:
            print('No input signal passed in for sampling, using recorded data')
            data = np.load('input_signal.npz')
            qs = data['q']
            dqs = data['dq']
            print(np.array(qs).shape)
            [qs, dqs] = generate_scaled_inputs(q=qs, dq=dqs, in_index=in_index)
            input_signal = np.hstack((qs, dqs))
            input_signal_len = len(input_signal)
            print('input_signal_length: ', input_signal_len)
        ii = 0
        same_count = 0
        prev_index = 0
        while (input_signal.shape[0] > n_neurons):
            if ii%1000 == 0:
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

            if same_count == 50:
                same_count = 0
                thresh += 0.001
                print('All values are within threshold, but not at target size.')
                print('Increasing threshold to %.4f' %thresh)
            prev_index = n_indices

        first_time = True
        while (input_signal.shape[0] != n_neurons):
            if first_time:
                print('Too many indices removed, appending random entries to'
                        + ' match dimensionality')
                print('shape: ', input_signal.shape)
            first_time = False
            row = np.random.randint(input_signal.shape[0])
            input_signal = np.vstack((input_signal, input_signal[row]))

        print(input_signal.shape)
        print('thresh: ', thresh)
        if use_spherical:
            encoders = convert_to_spherical(input_signal)
        else:
            encoders = np.array(input_signal)

    # seccessive run so load the encoders used for run 0
    # else:
    #     print('Loading encoders used for run 0...')
    #     encoders = self.data_handler.load(params=['encoders'],
    #             save_location='%s/%s/parameters/dynamics_adaptation/'
    #                             %(self.test_group, self.test_name))['encoders']
    np.savez_compressed('encoders-backup.npz', encoders=encoders)
    encoders = np.array(encoders)
    print(encoders.shape)
    return encoders

def generate_scaled_inputs(q, dq, in_index):
    '''
    pass q dq in as time x dim shape
    accepts the 6 joint positions and velocities of the jaco2 and does the
    mean subtraction and scaling. Can set which joints are of interest with
    in_index, if it is not passed in the self.in_index instantiated in
    __init_network__ will be used

    returns two n x 6 lists scaled, one for q and one for dq

    '''
    # check if we received a 1D input (one timestep) or a 2D input (list of
    # inputs over time)
    # if np.squeeze(q)[0] > 1 and np.squeeze(q)[1] > 1:
    #     print('Scaling list of inputs')
    qs = q.T
    dqs = dq.T
    #print('raw q: ', np.array(qs).T.shape)

    # add bias to joints 0 and 4 so that the input signal doesn't keep
    # bouncing back and forth over the 0 to 2*pi line
    qs[0] = (qs[0] + np.pi) % (2*np.pi)
    qs[4] = (qs[4] + np.pi) % (2*np.pi)

    MEANS = {  # expected mean of joint angles / velocities
        # shift from 0-2pi to -pi to pi
        'q': np.array([3.20, 2.14, 1.52, 4.68, 3.00, 3.00]),
        'dq': np.array([0.002, -0.117, -0.200, 0.002, -0.021, 0.002]),
        }
    SCALES = {  # expected variance of joint angles / velocities
        'q': np.array([0.2, 1.14, 1.06, 1.0, 2.8, 0.01]),
        'dq': np.array([0.06, 0.45, 0.7, 0.25, 0.4, 0.01]),
        }

    for pp in range(0, 6):
        qs[pp] = (qs[pp] - MEANS['q'][pp]) / SCALES['q'][pp]
        dqs[pp] = (dqs[pp] - MEANS['dq'][pp]) / SCALES['dq'][pp]

    qs = qs
    dqs = dqs
    scaled_q = []
    scaled_dq = []
    #print(in_index)
    for ii in in_index:
        scaled_q.append(qs[ii])
        scaled_dq.append(dqs[ii])
    scaled_q = np.array(scaled_q).T
    scaled_dq = np.array(scaled_dq).T
    #print('scaled q: ', np.array(scaled_q).shape)

    return [scaled_q, scaled_dq]

def convert_to_spherical(input_signal):
    """
    converts an input signal of shape time x N_joints and converts to
    spherical
    """
    #print('IN: ', input_signal.shape)
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
    #print('OUT: ', np.array(spherical).shape)
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
    input_signal = np.load('input_signal.npz')

# TODO: need to clean this up in the final form
# default in dynadapt using triangular intercepts, if using them keep
# intercepts=None, and pass in your intercepts_bounds and intercepts_mode
# if you want to manually set your intercepts you can pass them in directly
# here and intercepts_mode and intercepts_bounds will be ignored

n_neurons = 1000
n_ensembles = 45
use_spherical = True
n_inputs = 11
n_outputs = 5
adapt_input = [True, True, True, True, True, False]
in_index = np.arange(6)[adapt_input]
seed = 0

intercepts = signals.AreaIntercepts(
    dimensions=n_inputs,
    base=signals.Triangular(-0.9, -0.9, 0.0))

rng = np.random.RandomState(seed)
intercepts = intercepts.sample(n_neurons, rng=rng)
intercepts = np.array(intercepts)


# Define your encoders
# Leave input_signal as None and the function will load from the npz file,
# hacky but how it is for now
encoders = generate_encoders(input_signal=None, thresh=0.008,
        n_neurons=n_neurons*n_ensembles, use_spherical=use_spherical)

encoders = encoders.reshape(n_ensembles, n_neurons, n_inputs)

data = np.load('input_signal.npz')
qs = data['q']
dqs = data['dq']
[input_q, input_dq] = generate_scaled_inputs(
        q=qs, dq=dqs, in_index=in_index)
input_signal = np.hstack((input_q, input_dq))

if use_spherical:
    input_signal = convert_to_spherical(input_signal)

plt_learning = PlotLearningProfile(test_group=test_group,
        test_name=test_name, db_name=db_name, use_cache=True,
        #intercepts_bounds=[-0.5, -0.1], intercepts_mode=-0.4,
        encoders=encoders, intercepts=intercepts, use_spherical=False,
        n_inputs=n_inputs, n_outputs=n_outputs, n_neurons=n_neurons,
        n_ensembles=n_ensembles)

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
