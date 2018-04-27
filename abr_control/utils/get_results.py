import os
import numpy as np
from process_data import ProcessData
from generate_ideal_trajectories import GenerateTrajectory
from plot_error import PlotError
from plot_trajectory import PlotTrajectory

# TODO: either use saved tests to run or pass to everything as parameter
# currently have both happening in plot error and process respectively

def proc_and_plot(test_set_name, target_xyz, reaching_t, test_info,
        regenerate=False, plot_error=True, plot_trajectory=True,
        show_plot=True, **kwargs):
    #TODO: add the parameters in the functions below not listed as parameters
    #to kwargs check
    """
    Takes in a list of tests to process and plot

    Parameters
    ----------
    test_info: 2D list of strings and ints
        The list of tests which you want processed and plotted
        The following format is used...
        [
        ['weighted' or 'no_weight', 'backend', 'test_name', n_runs, n_sessions,
        'test_set']
        ]
    test_set_name: string
        the name of the group of tests you are plotting
        this will be used when naming the pdfs created
    target_xyz: 2D np.array of floats
        the target(s) used in the tests being plotted
    reaching_t: float
        the length of time for each test
    regenerate: Boolean, Optional (default: False)
        True to regenerate processed data
        False to try and load previously processed data
    plot_error: Boolean, Optional (default: True)
        True to generate error results
    plot_trajectory: Boolean, Optional (default: True)
        True to generate trajectory results
    show_plot: Boolean, Optional (default: True)
        True to show results at the end of the script
        if using automated email script, set this to false or else the plot
        will need to be closed manually before the results can be emailed
    """

    pd=ProcessData()
    gt=GenerateTrajectory()
    pe=PlotError()
    pt=PlotTrajectory()

    # Create processed data folder if it does not exist
    if not os.path.exists('proc_data'):
        os.makedirs('proc_data')

    # Save what data is to be plotted
    np.savez_compressed('proc_data/%s'%test_set_name, test_info=test_info)

    # Generate ideal trajectory with the desired test reaching times
    print('Generating Ideal Trajectory to Given Targets...')
    [u_track_ideal, x_track_ideal] = gt.get_ideal(reaching_t=reaching_t,
            target_xyz=target_xyz)

    # Process data
    print('Processing Data...')
    pd.get_set(regenerate=regenerate, test_info=test_info, n_targets=len(target_xyz),
            reach_time_per_target=reaching_t)

    if plot_error:
        # Plot error
        print('Plotting Error...')
        pe.get_error_plot(test_set_name=test_set_name, plot_total_error=False,
                plot_vs_ideal=True, n_targets=len(target_xyz),
                reaching_time_per_target=reaching_t, show_plot=show_plot)

    if plot_trajectory:
        # Plot trajectory
        print('Plotting Trajectories...')
        pt.get_trajectory_plot(test_set_name=test_set_name, target_xyz=target_xyz,
                reaching_t=reaching_t, use_recorded=True, session_to_plot=0,
                run_to_plot=min(test_info[:,3]), show_plot=show_plot)
