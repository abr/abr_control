import numpy as np
import os
from process_data import ProcessData
from generate_ideal_trajectories import GenerateTrajectory
from plot_error import PlotError
from plot_trajectory import PlotTrajectory

# TODO: either use saved tests to run or pass to everything as parameter
# currently have both happening in plot error and process respectively

pd=ProcessData()
gt=GenerateTrajectory()
pe=PlotError()
pt=PlotTrajectory()

unweighted_reach = False
weighted_reach = True
sim_wear = False
if unweighted_reach + weighted_reach + sim_wear > 1:
    print('Can only plot one at a time')

if unweighted_reach:
    # name of test set
    test_set_name = 'baseline_1pt_reach'
    target_xyz = np.array([[.57, .03, .87]])

    # reaching_dt = 10
    # test_info = [
    #              ['no_weight', 'pd', 'unweighted_5', 50, 1],
    #              ['no_weight', 'nengo/1000_neurons_x_1',
    #                  'unweighted_13', 100,2],
    #              ['no_weight', 'nengo_spinnaker/1000_neurons_x_50',
    #                  'unweighted_18', 50,1],
    #              ]


    reaching_dt = 3
    test_info = [
                 ['no_weight', 'pd', 'unweighted_4', 50, 5],
                 ['no_weight', 'pid', 'unweighted_19', 50, 5],
                 ['no_weight', 'nengo_spinnaker/1000_neurons_x_50',
                     'unweighted_21', 50,5],
                 ]

if weighted_reach:
    # name of test set
    test_set_name = '2lb_reach'
    target_xyz = np.array([[.57, .03, .87]])
    reaching_dt = 3
    test_info = [
                 ['weighted', 'pd', 'weighted_16', 50, 5],
                 ['weighted', 'pid', 'weighted_8', 50, 5],
                 ['weighted', 'nengo_spinnaker/1000_neurons_x_50', 'weighted_14',
                     50,5]
                 ]
if sim_wear:
    # name of test set
    test_set_name = 'simulated_wear'
    target_xyz = np.array([
        [.56, -.09, .95],
        [.12, .15, .80],
        [.80, .26, .61],
        [.38, .46, .81],
        [.0, .0, 1.15]
        ])
    reaching_dt = 4
    test_info = [
                 ['no_weight', 'pd', 'pd_5pt_baseline', 50, 5],
                 ['no_weight', 'pd', 'wear_124', 50, 5],
                 ['no_weight', 'pid', 'wear_125', 50, 5],
                 ['no_weight', 'nengo/1000_neurons_x_1', 'wear_120', 50,5],
                 ['no_weight', 'nengo/1000_neurons_x_50', 'wear_121', 50,5],
                 ['no_weight', 'nengo_spinnaker/1000_neurons_x_50', 'wear_123',
                     50,5],
                 ['no_weight', 'nengo_spinnaker/1000_neurons_x_1', 'wear_122',
                     50,5]
                 ]

# Create processed data folder if it does not exist
if not os.path.exists('proc_data'):
    os.makedirs('proc_data')

# Save what data is to be plotted
np.savez_compressed('proc_data/%s'%test_set_name, test_info=test_info)

# Generate ideal trajectory with the desired test reaching times
print('Generating Ideal Trajectory to Given Targets...')
[u_track_ideal, x_track_ideal] = gt.get_ideal(reaching_dt=reaching_dt,
        target_xyz=target_xyz)

# Process data
print('Processing Data...')
pd.get_set(regenerate=False, test_info=test_info, n_targets=len(target_xyz),
        reach_time_per_target=reaching_dt)

# Plot error
print('Plotting Error...')
pe.get_error_plot(test_set_name=test_set_name, plot_total_error=True,
        plot_vs_ideal=True, n_targets=len(target_xyz),
        reaching_time_per_target=reaching_dt)

# Plot trajectory
print('Plotting Trajectories...')
for ii in range(29,30):
    pt.get_trajectory_plot(test_set_name=test_set_name, target_xyz=target_xyz,
            reaching_dt=reaching_dt, use_recorded=True, session_to_plot=0,
            run_to_plot=ii, show_plot=False)
