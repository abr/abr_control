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

just_plot = False
use_spherical = True
test_group = '1lb_random_target'
test_name = 'nengo_cpu_18_0'
db_name = 'dewolf2018neuromorphic'
loc = '/%s/%s/'%(test_group, test_name)
use_cache = True
n_runs = 10
n_bins = 50
n_neurons = 20000
# [proportion_neurons_active, proportion_of_time_active]
max_activity = [0.1, 0]
min_activity = [0, 0.6]

if use_spherical:
    save_name = 'neuron_tuning_spherical'
else:
    save_name = 'neuron_tuning'

dat = DataHandler(use_cache=use_cache, db_name=db_name)

# Create list of all possible intercepts
# the stop value does not get included, so we use 1.0 instead of 0.9
intercept_range = np.arange(-.9, 1.0, .1)
mode_range = np.arange(-.9, 1.0, .2)

intercepts = np.array(np.meshgrid(intercept_range, intercept_range)).T.reshape(-1, 2)

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

# Create the ideal distribution
bins = np.linspace(0,1,n_bins)
n_line_points = int(min_activity[1] * n_bins)
n_zero_points = n_bins-n_line_points-1

# want 10% activity at 0 prop time and <5% at 0.6 prop time
ideal_dist = np.hstack((np.linspace(max_activity[0]*n_neurons,
    min_activity[0]*n_neurons, n_line_points),
                       np.zeros(n_zero_points)))

# Run through the intercepts in simulation to get the neural activity
if not just_plot:
    for ii ,intercept in enumerate(intercepts) :
        print(ii)
        plt_learning = PlotLearningProfile(test_group=test_group,
                test_name=test_name, db_name=db_name, use_cache=use_cache,
                intercepts_bounds=intercept[:2], intercepts_mode=intercept[2],
                use_spherical=use_spherical)

        session = 0
        s = t.time()
        active = []
        neurons_active = []
        prop_time = []
        diff_active = []
        sum_diff = []

        input_signal = []
        times = []
        for run in range(0, n_runs):
            run_data = dat.load(params=['input_signal', 'time'],
                    save_location='%ssession%03d/run%03d'
                    %(loc, session, run))
            training_data = dat.load(params=['adapt_input'],
                    save_location='%s/parameters/training' %loc)
            adapt_input = training_data['adapt_input']
            input_signal.append(run_data['input_signal'])
            times.append(run_data['time'])
            #ideal_dist = np.linspace(

        input_signal = np.vstack(input_signal)
        times = np.hstack(times)
        act = plt_learning.plot_activity(input_signal=input_signal, time=times,
                save_num=run, #input_joints=adapt_input,
                getting_ideal_intercepts=True)

        # save the activity data
        active.append(act)

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
                'ideal_dist': ideal_dist, 'sum_diff' : sum_diff}
        dat.save(data=time_active, save_location='%s/%05d'%(save_name,ii),
                overwrite=True)

        print(t.time()-s)
        print('~~~~~~~~~~~~~~')
        loop = (t.time()-s)/60
        len_remaining = len(intercepts)-ii
        print('Loop time: ', loop)
        print('Possibilities remaining: ', len_remaining)
        print('Approximate time remaining')
        print(loop*len_remaining, ' minutes')

# Plot the activity for the 5 sets of intercepts with the least deviation from
# the ideal
sum_diff = []
neurons_active = []
prop_time = []
intercepts_load = []
modes = []
plt.figure()
min_to_plot=10
for ii ,intercept in enumerate(intercepts) :
    print('intercept:',intercept)
    data = dat.load(params=['intercepts_bounds', 'intercepts_mode', 'diff_active',
            'neurons_active', 'prop_time', 'sum_diff'],
            save_location='%s/%05d'%(save_name,ii))

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

    #plt.hist(act, bins=bins)
#min_to_plot = len(sum_diff)
indices = np.array(sum_diff).argsort()[:min_to_plot]
print('Plotting...')
for ii in range(0, min_to_plot):
    ind = indices[ii]
    # print('IND: ', ind)
    # print(np.array(prop_time[ind]).shape)
    plt.plot(np.squeeze(prop_time[ind]), np.squeeze(neurons_active[ind]),
            label='%i: %f \n%s: %s'%
            (ind, sum_diff[ind], intercepts_load[ind], modes[ind]))

plt.plot(np.squeeze(prop_time[0]), ideal_dist, label='ideal')
plt.legend()
plt.show()
