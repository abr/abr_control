#NOTE: THIS CODE WAS HACKED TO TAKE IN Q AND DQ AND DO ALL OF THE CONVERSION TO INPUT
# SIGNAL HERE, THE PLOT OF THE RESULTS IS ODD (LOOKS LIKE THE HISTOGRAM SPIKES), MAKE
# SURE TO CHANGE BACK TO READING IN AND USING RECORDED INPUT SIGNAL WHEN YOU USE THIS
# IT LOOKS LIKE INSTEAD OF APPENDING EACH NEW LIST TO THE SAVE LIST, THE ENTIRE SAVE
# LIST GETS APPENDED EVERY TIME (entry 0 i 1,n, entry 1 is 2,n, entry 2 is 3,n etc)
#                   - past Pawel (got your back)
#NOTE: 177 and 207 WHERE DATA IS SAVED AND LOADED IS HACKED TO SAVE TO A NEW LOCATION
# WITH AND _2, CHANGE THIS OR ELSE YOU WILL GET ERRORS
"""
using the input from a provided test and the desired upper and lower limits of
a triangular distribution, the script will run through every possible set of
intercepts and modes. The sets that provide activity that matches the ideal the
closest will be plotted along with their intercept values.
"""
from abr_control.utils import DataHandler, PlotLearningProfile
import numpy as np
import time as t
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# TODO: HACKED IN BECAUSE IT'S FRIDAY NIGHT, DELETE THIS FUNCTION LATER
def convert_to_spherical_fun(input_signal):
    x = input_signal
    pi = np.pi
    spherical = []
    print('COMING IN: ', np.array(input_signal).shape)

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
    print('GOING OUT: ', np.array(spherical).shape)
    return(spherical)


just_plot = True
use_spherical = True
# False if input being used is already in spherical
convert_to_spherical = False
test_group = 'friction_post_tuning'
test_name = 'nengo_loihi_friction_10_0'
db_name = 'dewolf2018neuromorphic'
save_db_name = 'parameter_tuning'
loc = '/%s/%s/'%(test_group, test_name)
use_cache = True
n_runs = 50
n_bins = 50
n_neurons = 10000
# the following are two points on a line that will be used to create the ideal
# distribution that will be compared against to find the closest match
# [proportion_neurons_active, proportion_of_time_active]
max_activity = [0.1, 0]
min_activity = [0, 0.6]

if use_spherical:
    save_name = '%s/%s/neuron_tuning_spherical'%(test_group, test_name)
else:
    save_name = '%s/%s/neuron_tuning'%(test_group, test_name)

dat = DataHandler(use_cache=use_cache, db_name=db_name)
dat2 = DataHandler(use_cache=use_cache, db_name=save_db_name)

# Create list of all possible intercepts
# the stop value does not get included, so we use 1.0 instead of 0.9
intercept_range = np.arange(-.9, 1.0, .1)
mode_range = np.arange(-.9, 1.0, .2)

intercepts = np.array(np.meshgrid(intercept_range, intercept_range)).T.reshape(-1, 2)

# get a list of all valid intercepts
valid = []
rej = []
for vals in intercepts:
    vals[0] = round(vals[0], 1)
    vals[1] = round(vals[1], 1)
    if vals[0] < vals[1]:
        for mode in mode_range:
            mode = round(mode, 1)
            if vals[0] <= mode and mode <= vals[1]:
                if use_spherical:
                    valid.append(np.array([vals[0],vals[1], mode]))
                else:
                    if vals[0] < 0 and vals[1] > 0:
                        valid.append(np.array([vals[0], vals[1], mode]))

valid = np.array(valid)
intercepts = valid
print('There are %i valid combinations of intercepts and modes'%len(valid))

# Create the ideal distribution
bins = np.linspace(0,1,n_bins)
n_line_points = int(min_activity[1] * n_bins)
n_zero_points = n_bins-n_line_points-1

# want 10% activity at 0 prop time and <5% at 0.6 prop time
ideal_dist = np.hstack((np.linspace(max_activity[0]*n_neurons,
    min_activity[0]*n_neurons, n_line_points),
                       np.zeros(n_zero_points)))

# Run through the intercepts in simulation to get the neural activity
#intercepts = [[-0.5, -0.4, -0.5]]
if not just_plot:
    session = 0
    s = t.time()
    active = []
    neurons_active = []
    prop_time = []
    diff_active = []
    sum_diff = []

    input_signal = []
    times = []
    dq = []
    q = []
    qdq = []
    #TODO: this doesn't need to be in a loop
    for run in range(0, n_runs):
        run_data = dat.load(params=['input_signal', 'time', 'q', 'dq'],
                save_location='%ssession%03d/run%03d'
                %(loc, session, run))
        training_data = dat.load(params=['adapt_input'],
                save_location='%s/parameters/training' %loc)
        adapt_input = training_data['adapt_input']
        times.append(run_data['time'])
        #NOTE USE THIS IF READING INPUT DIRECTLY
        input_signal.append(run_data['input_signal'])
        #NOTE: USE THIS IF CONVERTING FROM Q DQ
        qs = run_data['q'].T
        dqs = run_data['dq'].T

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
        for pp in range(0, 5):
            qs[pp] = (qs[pp] - MEANS['q'][pp]) / SCALES['q'][pp]
            dqs[pp] = (dqs[pp] - MEANS['dq'][pp]) / SCALES['dq'][pp]
        # q.append(qs)
        # dq.append(dqs)
        # print('q: ', np.array(qs).shape)
        # print('dq: ', np.array(dqs).shape)
        # print('h: ', np.hstack((qs,dqs)).shape)
        print('v: ', np.vstack((qs,dqs)).T.shape)
        converted = (convert_to_spherical_fun(input_signal=np.vstack((qs,dqs))))
        print('converted: ', np.array(converted).shape)
        qdq.append(np.array(converted).T)
        print('input: ', np.array(input_signal[-1]).shape)
        print('qdq: ', np.array(qdq[-1]).shape)

    print(np.array(input_signal).shape)
    print(np.array(qdq).shape)
    print(np.vstack(input_signal).shape)
    print(np.vstack(qdq).shape)
    input_signal = np.vstack(qdq)

    #NOTE: do this if using recorded input signal
    #input_signal = np.vstack(input_signal)
    times = np.hstack(times)
    # np.savez_compressed('input_data.npz', input_signal=input_signal, times=times)
    # print("SAAAAAAAAAAAAAAAAAAAAAAAVED")
    # import time
    # time.sleep(100000)

    for ii ,intercept in enumerate(intercepts) :
        print(ii)
        plt_learning = PlotLearningProfile(test_group=test_group,
                test_name=test_name, db_name=db_name, use_cache=use_cache,
                intercepts_bounds=intercept[:2], intercepts_mode=intercept[2],
                use_spherical=convert_to_spherical)

        (act, activities) = plt_learning.plot_activity(input_signal=input_signal, time=times,
                save_num=run, #input_joints=adapt_input,
                getting_ideal_intercepts=True)

        # save the activity data
        active.append(act)

        # check how many neurons are never active
        print('ACTIVITIES SHAPE: ', np.array(activities).shape)
        num_inactive = 0
        num_active = 0
        times_neuron_fires = []
        for ens in activities:
            ens = ens.T
            for nn, neuron in enumerate(ens):
                # print('shape2: ', np.array(neuron).shape)
                # break
                if np.sum(ens[nn]) == 0:
                    num_inactive += 1
                else:
                    num_active += 1
                times_neuron_fires.append(np.sum(ens[nn]))
        print('Number of neurons inactive: ', num_inactive)
        print('Number of neurons active: ', num_active)
        # fig = plt.figure()
        # ax = fig.add_subplot(111)
        # ax.plot(np.sort(np.array(times_neuron_fires)),
        #     label='Number of neurons inactive: %i'% num_inactive)
        # ax.set_ylabel('Number of times fired')
        # ax.set_xlabel('Neuron (sorted low to high)')
        # ax.set_ylim([-100,len(ens.T)+100])
        # ax.plot(np.ones(len(times_neuron_fires))* len(ens.T), 'r',
        # label='Number of timesteps')
        # ax.legend()
        # plt.show()

        # save the data for the line plot of the histogram
        y, bins_out = np.histogram(act, bins=bins)
        centers = 0.5*(bins_out[1:]+bins_out[:-1])
        prop_time.append(centers)
        neurons_active.append(y)

        # get the difference from the ideal distribution
        diff_active.append(ideal_dist-y)
        sum_diff.append(np.sum(abs(ideal_dist-y)))

        #sum_diff = np.sum(sum_diff)
        time_active = {'raw_active': active, 'intercepts_bounds': intercept[:2],
                'intercepts_mode': intercept[2], 'diff_active': diff_active,
                'neurons_active': neurons_active, 'prop_time': prop_time,
                'ideal_dist': ideal_dist, 'sum_diff' : sum_diff, 'test_group':
                test_group, 'num_active': num_active, 'num_inactive': num_inactive}
        dat2.save(data=time_active, save_location='%s_3/%05d'%(save_name,ii),
                overwrite=True)

        print(t.time()-s)
        print('~~~~~~~~~~~~~~')
        loop = (t.time()-s)/60
        len_remaining = len(intercepts)-ii
        print('Loop time: ', loop)
        print('Possibilities remaining: ', len_remaining)
        print('Approximate time remaining')
        print(loop*len_remaining, ' minutes')
        # delete class instance to avoid filling up memory
        del plt_learning

# Plot the activity for the 5 sets of intercepts with the least deviation from
# the ideal
sum_diff = []
neurons_active = []
prop_time = []
intercepts_load = []
modes = []
num_active = []
num_inactive = []

plt.figure()
min_to_plot=10
for ii ,intercept in enumerate(intercepts) :
    print('intercept:',intercept)
    data = dat2.load(params=['intercepts_bounds', 'intercepts_mode', 'diff_active',
            'neurons_active', 'prop_time', 'sum_diff', 'num_active', 'num_inactive'],
            save_location='%s_3/%05d'%(save_name,ii))

    sum_diff0 = data['sum_diff']
    sum_diff.append(sum_diff0[0])

    prop_time0 = data['prop_time']
    prop_time.append(prop_time0)

    neurons_active0 = data['neurons_active']
    neurons_active.append(neurons_active0)

    intercepts_bounds0 = data['intercepts_bounds']
    intercepts_load.append(intercepts_bounds0)

    modes0 = data['intercepts_mode']
    modes.append(modes0)

    num_active.append(data['num_active'])
    num_inactive.append(data['num_inactive'])

    #plt.hist(act, bins=bins)
#min_to_plot = len(sum_diff)
indices = np.array(sum_diff).argsort()[:min_to_plot]
print('Plotting...')
for ii in range(0, min_to_plot):
    ind = indices[ii]
    print(np.array(sum_diff[ind]).shape)
    # print('IND: ', ind)
    # print(np.array(prop_time[ind]).shape)
    plt.figure()
    plt.title('Active: %i | Inactive: %i'%(num_active[ind], num_inactive[ind]))
    plt.plot(np.squeeze(prop_time[ind]), np.squeeze(neurons_active[ind]),
            label='%i: %f \n%s: %s'%
            (ind, sum_diff[ind], intercepts_load[ind], modes[ind]))

    plt.plot(np.squeeze(prop_time[0]), ideal_dist, label='ideal')
    plt.legend()
    plt.show()
    plt.savefig('images3/%i.png'%ii)
    plt.close()

# unique = {}
# unique['labels'] = []
# unique['intercepts'] = []
# for ii, intercepts in enumerate(intercepts_load):
#     if '%.1f, %.1f'%(intercepts[0], intercepts[1]) not in unique['labels']:
#         unique['labels'].append('%.1f, %.1f'%(intercepts[0], intercepts[1]))
#         unique['intercepts'].append(intercepts)
#
# for mm, unique_bound in enumerate(unique['intercepts']):
#     plt.figure
#     for ind, intercepts in enumerate(intercepts_load):
#         if intercepts[0] == unique_bound[0] and intercepts[1] == unique_bound[1]:
#             plt.plot(np.squeeze(prop_time[ind]), np.squeeze(neurons_active[ind]),
#                     label='%i: %f \n%s: %s'%
#                     (ind, sum_diff[ind], intercepts_load[ind], modes[ind]))
#     plt.legend()
#     plt.savefig('images2/%i.png'%mm)
# plots in a grid to separate by intercepts
# bounds = [-0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1,
#         0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
#
# plt.figure()
# ax = []
# sub_nums = []
# matplotlib.rcParams['lines.linewidth'] = 0.1
# matplotlib.rcParams.update({'font.size': 1})
# matplotlib.rcParams['axes.titlesize'] = 1
# for ii, intercepts in enumerate(intercepts_load):
#
#     left = intercepts[0]
#     right = intercepts[1]
#     # get the subplot location in x
#     sub_x = bounds.index(right)
#     # get the subplot location in y
#     sub_y = bounds.index(left)
#     # get subplot index
#     # +1 because base 1, not 0
#     sub_num = (sub_x+1) + (len(bounds)*sub_y)
#     if sub_num not in sub_nums:
#         sub_nums.append(sub_num)
#         ax.append(plt.subplot(len(bounds), len(bounds), (sub_x+1) + sub_num))
#         ax_index = len(ax)-1
#     else:
#         ax_index = sub_nums.index(sub_num)
#
#     # if sub_num < len(bounds):
#     #     plt.title('Right: %f' % right)
#     # if (sub_num-1)%19 == 0:
#     #     plt.ylabel('Left: %f' % left)
#
#     ax[ax_index].plot(np.squeeze(prop_time[ii]), np.squeeze(neurons_active[ii]),
#             label=modes[ii])
#             # label='%i: %f \n%s: %s'%
#             # (ind, sum_diff[ii], intercepts_load[ii], modes[ii]))
#     ax[ax_index].set_title('Left: %0.1f | Right: %0.1f'%(left, right))
#     plt.legend()
# for a in ax:
#     a.set_xticks([])
#     a.set_yticks([])
# plt.tight_layout()
# plt.savefig('destination_path.svg', format='svg', dpi=100000)
#plt.show()
