import numpy as np
import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
from matplotlib.colors import colorConverter as cc
import os

from legend_object import LegendObject
from abr_control.utils import DataHandler

class PlotError():
    def __init__(self):
        pass

    def plot_mean_and_CI(self, data, color_mean=None,
                         color_shading=None, marker='--',
                         scaling_factor=1):
        # print('color shading: ', color_shading)
        # print('color mean: ', color_mean)
        # print('shape: ', data['mean'].shape)
        self.a.fill_between(range(data['mean'].shape[0]),
                         scaling_factor*data['upper_bound'],
                         scaling_factor*data['lower_bound'],
                         color=color_shading,
                         alpha=.5)
        if len(data['mean']) == 1:
            self.a.plot(np.ones(30) *data['mean'], color_mean)
        else:
            #print('color_mean: ', color_mean)
            self.a.plot(scaling_factor*data['mean'], marker, color=color_mean)

        #print('Average of last 5 trials: %.3f' % np.mean(data['mean'][-5:]))

    def plot_data(self, data, legend_labels=None, fig_title='Error',
                  x_label='Run Number', y_label='Error', xlim=None, ylim=None,
                  show_plot=True, save_figure=False, colors=None, legend_loc=1,
                  fig_obj=None, fig_size=[8,3], scaling_factor=1,
                  clear_plot=False, statistical=True):

        print('***********************using legend labels of: ', legend_labels)
        if fig_obj is None:
            fig = plt.figure(1, figsize=(fig_size[0], fig_size[1]))
            #fig.tight_layout()
            self.a = fig.add_subplot(111)
        else:
            self.a = fig_obj

        self.a.set_title(fig_title, fontsize=14)
        self.a.set_xlabel(x_label, fontsize=12)
        self.a.set_ylabel(y_label, fontsize=12)

        if xlim is not None:
            self.a.set_xlim(xlim[0], xlim[1])
        if ylim is not None:
            self.a.set_ylim(ylim[0], ylim[1])

        if colors is None:
            colors = []
            for ii in range(0,len(data)):
                colors.append(np.around(np.random.rand(3,1), decimals=1))

        print('plotting colors post: ', colors)
        markers = ['--',] * len(colors)

        # if plotting mean and ci data
        if statistical:
            for ii in range(0, len(data)):
                # print('col %i'%ii)
                # print(colors[ii])
                # print(markers)
                self.plot_mean_and_CI(data[ii], color_mean=colors[ii],
                                      color_shading=colors[ii], marker=markers[ii],
                                      scaling_factor=scaling_factor)
        # otherwise plot the data directly as passed in
        else:
            for ii, test in enumerate(data):
                self.a.plot(scaling_factor*test, marker, color=color_mean)

        #self.a.grid()

        bg = np.array([1, 1, 1])  # background of the legend is white
        # with alpha = .5, the faded color is the average of the background and color
        colors_faded = [(np.array(cc.to_rgb(color)) + bg) / 2.0 for color in colors]

        if legend_labels is not None:
            handler_map = {}
            for ii in range(0, len(colors)):
                handler_map[ii] = LegendObject(colors[ii], colors_faded[ii])

            self.a.legend(range(7), legend_labels,
                       handler_map=handler_map,
                       loc=legend_loc, prop={'size':10})


        if save_figure:
            if not os.path.exists('figures'):
                os.makedirs('figures')
            plt.savefig('figures/%s.pdf'%(fig_title))

        if show_plot:
            plt.show()

        if clear_plot:
            self.a.clear()

    def get_error_plot(self, test_group, test_list, show_plot=True,
            save_figure=False, fig_title='Error',
            x_label='Run Number', y_label='Error', xlim=None, ylim=None,
            legend_labels=None, colors=None, legend_loc=1, use_cache=True,
            db_name=None, order_of_error=[0], sum_errors=True,
            scaling_factor=1, clear_plot=False):

        dat = DataHandler(use_cache=use_cache, db_name=db_name)
        # create plot here, this allows for direct call of the plotting
        # function from exterior scripts that have already created a figure
        # The main use for this is for integrating into the plotting GUI
        orders = ['position', 'velocity', 'acceleration', 'jerk']

        if legend_labels is None:
            legend_labels = test_list

        data = []

        #TODO: cycle through all the orders of error passed in and sum them
        for ii, test in enumerate(test_list) :
            temp_data = []
            for order in order_of_error:
                save_location = '%s/%s/proc_data/%s'%(test_group, test,
                        orders[order])
                temp_data.append(dat.load(params=['mean', 'upper_bound', 'lower_bound'],
                   save_location=save_location))

            mean = np.zeros(len(temp_data[0]['mean']))
            upper = np.zeros(len(temp_data[0]['upper_bound']))
            lower = np.zeros(len(temp_data[0]['lower_bound']))

            for error_type in temp_data:
                mean += error_type['mean']
                upper += error_type['upper_bound']
                lower += error_type['lower_bound']

            # we divide by the number or error terms since they scale data from
            # 0-1, summing error will increase this range (two orders of error
            # will have a range of 0-2)
            data.append({'mean': mean/len(order_of_error), 'lower_bound':
                lower/len(order_of_error), 'upper_bound':
                upper/len(order_of_error)})


        self.plot_data(data=data, legend_labels=legend_labels,
                fig_title=fig_title, x_label=x_label, y_label=y_label,
                xlim=xlim, ylim=ylim, show_plot=show_plot,
                save_figure=save_figure, colors=colors, legend_loc=legend_loc,
                scaling_factor=scaling_factor, clear_plot=clear_plot,
                statistical=True)

    # def get_traj_sub_plot(self, test_group, test_list, show_plot=True,
    #         save_figure=False, fig_title='Error',
    #         x_label='Run Number', y_label='Error', xlim=None, ylim=None,
    #         legend_labels=None, colors=None, legend_loc=1, use_cache=True,
    #         db_name=None, order_of_error=[0], sum_errors=True,
    #         scaling_factor=1, clear_plot=False):

    #     dat = DataHandler(use_cache=use_cache, db_name=db_name)

    #     if legend_labels is None:
    #         legend_labels = test_list

    #     data = []

    #     #TODO: cycle through all the orders of error passed in and sum them
    #     for ii, test in enumerate(test_list) :
    #         temp_data = []
    #         save_location = '%s/%s/proc_data/%s'%(test_group, test,
    #                 orders[order])
    #         temp_data.append(dat.load(params=['ee_xyz'],
    #            save_location=save_location))

    #         ee_xyz = np.zeros(len(temp_data[0]['mean']))

    #         for error_type in temp_data:
    #             mean += error_type['mean']
    #             upper += error_type['upper_bound']
    #             lower += error_type['lower_bound']

    #         # we divide by the number or error terms since they scale data from
    #         # 0-1, summing error will increase this range (two orders of error
    #         # will have a range of 0-2)
    #         data.append({'mean': mean/len(order_of_error), 'lower_bound':
    #             lower/len(order_of_error), 'upper_bound':
    #             upper/len(order_of_error)})


    #     self.plot_data(data=data, legend_labels=legend_labels,
    #             fig_title=fig_title, x_label=x_label, y_label=y_label,
    #             xlim=xlim, ylim=ylim, show_plot=show_plot,
    #             save_figure=save_figure, colors=colors, legend_loc=legend_loc,
    #             scaling_factor=scaling_factor, clear_plot=clear_plot,
    #             statistical=False)
