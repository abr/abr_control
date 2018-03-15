import numpy as np
import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
from matplotlib.colors import colorConverter as cc
import os

from legend_object import LegendObject

#TODO: save pdf results to cache_dir
class PlotError():
    def __init__(self):
        pass

    def plot_mean_and_CI(self, data, color_mean=None, color_shading=None):
        plt.fill_between(range(data['mean'].shape[0]),
                         data['upper_bound'],
                         data['lower_bound'],
                         color=color_shading,
                         alpha=.5)
        if len(data['mean']) == 1:
            plt.plot(np.ones(30) *data['mean'], color_mean)
        else:
            plt.plot(data['mean'], color_mean)

    def plot_data(self, proc_data, legend_labels, fig_title='Error Over Runs',
                  x_axis='Run Number', y_axis='Error', xlim=50):
        fig = plt.figure(1, figsize=(8, 3))
        plt.title(fig_title)
        plt.xlabel(x_axis)
        plt.ylabel(y_axis)
        colors = ['k', 'b', 'g', 'r', 'y', 'm', 'c']

        for ii in range(0, len(proc_data)):
            self.plot_mean_and_CI(proc_data[ii], color_mean=colors[ii] + '--',
                    color_shading=colors[ii])
        plt.grid()
        plt.xlim([0, xlim])

        bg = np.array([1, 1, 1])  # background of the legend is white
        # with alpha = .5, the faded color is the average of the background and color
        colors_faded = [(np.array(cc.to_rgb(color)) + bg) / 2.0 for color in colors]

        plt.legend(range(7), legend_labels,
                   handler_map={
                       0: LegendObject(colors[0], colors_faded[0]),
                       1: LegendObject(colors[1], colors_faded[1]),
                       2: LegendObject(colors[2], colors_faded[2]),
                       3: LegendObject(colors[3], colors_faded[3]),
                       4: LegendObject(colors[4], colors_faded[4]),
                       5: LegendObject(colors[5], colors_faded[5]),
                       6: LegendObject(colors[6], colors_faded[6]),
                    })

        plt.tight_layout()
        if not os.path.exists('figures'):
            os.makedirs('figures')
        plt.savefig('figures/%s.pdf'%fig_title)
        if self.show_plot:
            plt.show()

    def get_error_plot(self, test_set_name, plot_total_error=True, plot_vs_ideal=True,
            n_targets=1, reaching_time_per_target=3, show_plot=True):

        test_info = np.load('proc_data/%s.npz'%test_set_name)['test_info']
        test_info = np.array(test_info)
        self.show_plot = show_plot

        print('Plotting the Following from %s:'%test_set_name)
        print(test_info)

        proc_data_traj = []
        proc_data_err = []
        # legend_labels = []
        #legend_labels = ['Spin50k', 'chip','nengo 1e-4 old scale old data', 'nengo 1e-4 new scale',
        #'nengo 5-4 new scale', 'nengo 5-4 old scale', 'nengo 1e-4 old scale']
        #legend_labels = test_info[:,2]
        legend_labels = []
        xlims = []

        for ii in range(0, len(test_info)):
            backend = test_info[ii,1]
            test_name = test_info[ii,2]
            num_runs = int(test_info[ii,3])
            xlims.append(np.copy(num_runs))
            num_sessions = int(test_info[ii,4])
            legend_labels.append(np.copy(backend + ' ' + test_name))
            # legend_labels.append(np.copy((backend + '/' + test_name)))

            if plot_total_error:
                proc_data_err.append(np.load(
                    ('proc_data/%s/error_to_target_%i_x_%i_runs_dt0.0050_%itargets_'
                     + 'x_%isec_%s.npz')
                    %(test_name, num_sessions, num_runs, n_targets,
                        reaching_time_per_target, test_name)))

            if plot_vs_ideal:
                proc_data_traj.append(np.load(
                    ('proc_data/%s/trajectory_error_%i_x_%i_runs_dt0.0050_%itargets_'
                     + 'x_%isec_%s.npz')
                    %(test_name, num_sessions, num_runs, n_targets,
                        reaching_time_per_target, test_name)))


        x_lim = max(xlims)
        if plot_total_error:
            self.plot_data(proc_data=proc_data_err,
                      legend_labels=legend_labels,
                      fig_title=('%s:_Cumulative_Error_to_Target_Over_Runs'
                                 %test_set_name),
                      x_axis='Runs',
                      y_axis='Error [m$^2$]',
                      xlim=x_lim)

        if plot_vs_ideal:
            self.plot_data(proc_data=proc_data_traj,
                      legend_labels=legend_labels,
                      fig_title=('%s:_Trajectory_Error_Compared_to_Ideal'
                                 %test_set_name),
                      x_axis='Runs',
                      y_axis='Error [m$^2$]',
                      xlim=x_lim)
