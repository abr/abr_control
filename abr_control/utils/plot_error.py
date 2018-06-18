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
                         color_shading=None, marker='--'):
        print('shape: ', data['mean'].shape)
        self.a.fill_between(range(data['mean'].shape[0]),
                         100*data['upper_bound'],
                         100*data['lower_bound'],
                         color=color_shading,
                         alpha=.5)
        if len(data['mean']) == 1:
            self.a.plot(np.ones(30) *data['mean'], color_mean)
        else:
            print('color_mean: ', color_mean)
            self.a.plot(100*data['mean'], marker, color=color_mean)

        print('Average of last 5 trials: %.3f' % np.mean(data['mean'][-5:]))

    def plot_data(self, data, legend_labels=None, fig_title='Error',
                  x_label='Run Number', y_label='Error', xlim=None, ylim=None,
                  show_plot=True, save_figure=False, colors=None, legend_loc=1,
                  fig_obj=None, fig_size=[8,3]):

        if fig_obj is None:
            fig = plt.figure(1, figsize=(fig_size[0], fig_size[1]))
            fig.tight_layout()
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

        markers = ['--',] * len(colors)

        for ii in range(0, len(data)):
            self.plot_mean_and_CI(data[ii], color_mean=colors[ii],
                                  color_shading=colors[ii], marker=markers[ii])
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
            self.a.savefig('figures/%s.pdf'%(fig_title))

        if show_plot:
            self.a.show()

    def get_error_plot(self, test_group, test_list, show_plot=True,
            save_figure=False, fig_title='Error',
            x_label='Run Number', y_label='Error', xlim=None, ylim=None,
            legend_labels=None, colors=None, legend_loc=1, use_cache=True,
            order_of_error=[0]):

        dat = DataHandler(use_cache=use_cache)
        # create plot here, this allows for direct call of the plotting
        # function from exterior scripts that have already created a figure
        # The main use for this is for integrating into the plotting GUI
        orders = ['position', 'velocity', 'acceleration', 'jerk']

        if legend_labels is None:
            legend_labels = test_list

        data = []

        #TODO: cycle through all the orders of error passed in and sum them
        for ii, test in enumerate(test_list) :
            print(ii)
            print(test)
            save_location = '%s/%s/proc_data/%s'%(test_group, test,
                    orders[order_of_error])
            data.append(dat.load(params=['mean', 'upper_bound', 'lower_bound'],
                save_location=save_location))

        self.plot_data(data=data, legend_labels=legend_labels,
                fig_title=fig_title, x_label=x_label, y_label=y_label,
                xlim=xlim, ylim=ylim, show_plot=show_plot,
                save_figure=save_figure, colors=colors, legend_loc=legend_loc)
