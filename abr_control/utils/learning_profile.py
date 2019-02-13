"""
Accepts a Nengo network and input signal
Plots
1. rasterplot showing spikes for each neuron over time on one axis, and the
   input signal of the other
2. proportion of time active, the number of neurons active vs proportion
   of run time
3. proportion of neurons that are active over time
"""
import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
import seaborn
import numpy as np
import os

import nengo
import nengolib
from nengo.utils.matplotlib import rasterplot

class LearningProfile:
    def raster_plot(self, network, input_signal, ax, num_ens_to_raster=None):
        '''
        Accepts a Nengo network and runs a simulation with the provided input_signal
        Plots rasterplot onto ax object up to num_ens_to_raster ensembles
        if num_ens_to_raster is None, all ensembles will be plotted

        PARAMETERS
        ----------
        network: a Nengo network object
        input_signal: [time x input_dim] list
            the input used for the network sim
        ax: ax object
            used for the rasterplot
        num_ens_to_raster: int, Optional (Default: None)
            the number of ensembles to plot in the raster, if None all will be plotted
        '''
        # create probes to get rasterplot
        with network.nengo_model:
            if not hasattr(network.nengo_model, 'ens_probes'):
                network.nengo_model.ens_probes = []
                for ens in network.adapt_ens:
                    network.nengo_model.ens_probes.append(nengo.Probe(ens.neurons,
                            synapse=None))

        sim = nengo.Simulator(network.nengo_model)
        print('Running sim...')
        for ii, inputs in enumerate(input_signal):
            network.input_signal = inputs
            sim.run(time_in_seconds=0.001, progress_bar=False)
        print('Sim complete')

        probes = []
        print('Plotting spiking activity...')
        for ii, probe in enumerate(network.nengo_model.ens_probes):
            probes.append(sim.data[probe])
            # it can become hard to read the plot if there are too many ensembles
            # plotted onto one raster plot, let the user choose how many to plot
            if num_ens_to_raster is not None:
                if num_ens_to_raster == ii+1:
                    break

        probes = np.hstack(probes)
        time = np.ones(len(input_signal))
        ax = rasterplot(np.cumsum(time),probes, ax=ax)
        plt.ylabel('Neuron')
        plt.xlabel('Time [sec]')
        plt.title('Spiking Activity')

    def plot_prop_active_neurons_over_time(self, network, input_signal, ax, thresh=None):
        '''
        Accepts a Nengo network and checks the tuning curve responses to the input signal
        Plots the proportion of active neurons vs run time onto the ax object
        Returns the proportion active and the activities

        PARAMETERS
        ----------
        network: a Nengo network object
        input_signal: [time x input_dim] list
            the input used for the network sim
        ax: ax object
            used for the rasterplot
        thresh: float, Optional (Default: None)
            the values above and below which activities get set to 1 and 0, respectively
            When None, the default of the function will be used
        '''
        time = np.ones(len(input_signal))
        activities = self.get_activities(network=network, input_signal=input_signal)
        proportion_active = []
        for activity in activities:
            # axis=0 mean over time
            # axis=1 mean over neurons
            # len(activity.T) gives the number of neurons
            proportion_active.append(np.sum(activity, axis=1)/len(activity.T))

        proportion_active = np.sum(proportion_active,
                axis=0)/len(proportion_active)

        print('Plotting proportion of active neurons over time...')
        ax.plot(np.cumsum(time), proportion_active, label='proportion active')

        ax.set_title('Proportion of active neurons over time')
        ax.set_ylabel('Proportion Active')
        ax.set_xlabel('Time [sec]')
        ax.set_ylim(0, 1)
        plt.legend()
        return(proportion_active, activities)

    def plot_prop_time_neurons_active(self, network, input_signal, ax, thresh=None):
        '''
        Accepts a Nengo network and checks the tuning curves response to the input signal
        Plots the the number of active neurons vs proportion of run time onto the ax object
        Returns the time active and the activities

        PARAMETERS
        ----------
        network: a Nengo network object
        input_signal: [time x input_dim] list
            the input used for the network sim
        ax: ax object
            used for the rasterplot
        thresh: float, Optional (Default: None)
            the values above and below which activities get set to 1 and 0, respectively
            When None, the default of the function will be used
        '''
        time = np.ones(len(input_signal))
        activities = self.get_activities(network=network, input_signal=input_signal)
        time_active = []
        for activity in activities:
            # axis=0 mean over time
            # axis=1 mean over neurons
            # len(activity.T) gives the number of neurons
            time_active.append(np.sum(activity, axis=0)/len(activity))

        print('Plotting proportion of active time...')
        time_active = np.hstack(time_active)
        plt.hist(time_active, bins=np.linspace(0,1,100))
        ax.set_ylabel('Number of active neurons')
        ax.set_xlabel('Proportion of Time')

        num_inactive = 0
        num_active = 0
        times_neuron_fires = []
        for ens in activities:
            ens = ens.T
            for nn, neuron in enumerate(ens):
                if np.sum(ens[nn]) == 0:
                    num_inactive += 1
                else:
                    num_active += 1
                times_neuron_fires.append(np.sum(ens[nn]))
        ax.set_title('Proportion of time neurons are active\n'
            + 'Active: %i,  Inactive: %i'%(num_active, num_inactive))
        print('Number of neurons inactive: ', num_inactive)
        print('Number of neurons active: ', num_active)
        return (time_active, activities)

    def get_activities(self, network, input_signal, thresh=1e-5):
        '''
        Accepts a Nengo network and input signal and returns a list of the neural
        activities set to 1 or 0 based on the set thresh

        PARAMETERS
        ----------
        network: a Nengo network object
        input_signal: [time x input_dim] list
            the input used for the network sim
        thresh: float, Optional (Default: 1e-5)
            the values above and below which activities get set to 1 and 0, respectively
        '''

        activities = []
        print('Getting activities...')
        for ens in network.adapt_ens:
            _, activity = nengo.utils.ensemble.tuning_curves(ens,
                    network.sim, input_signal)
            activity[activity>thresh]=1
            activity[activity<=thresh]=0
            activities.append(np.copy(activity))
        return activities

    def generate_all(self, network, input_signal, ax=None, num_ens_to_raster=None,
            thresh=None, show_plot=True):
        """
        Plots the networks neural activity onto three subplots, showing the rasterplot,
        proportion of active neurons over time, and how many neurons were active over
        different proportions of run time

        PARAMETERS
        ----------
        network: a Nengo network object
        input_signal: [time x input_dim] list
            the input used for the network sim
        ax: list of 3 ax objects, Optional (Default: None)
            if the three ax objects are not provided, they will be created
        num_ens_to_raster: int, Optional (Default: None)
            the number of ensembles to plot in the raster, if None all will be plotted
        thresh: float, Optional (Default: None)
            the values above and below which activities get set to 1 and 0, respectively
            When None, the default of the function will be used
        show_plot: boolean, Optional (Default: True)
            whether to show the figure at the end of the script or not
        """

        if ax is None:
            plt.figure(figsize=(8,15))
            ax = []
            for ii in range(0,3):
                ax.append(plt.subplot(3,1,ii+1))

        self.raster_plot(
                network=network,
                input_signal=input_signal,
                ax=ax[0],
                num_ens_to_raster=num_ens_to_raster)

        self.plot_prop_active_neurons_over_time(
                network=network,
                input_signal=input_signal,
                ax=ax[1],
                thresh=thresh)

        self.plot_prop_time_neurons_active(
                network=network,
                input_signal=input_signal,
                ax=ax[2],
                thresh=thresh)

        if show_plot:
            plt.tight_layout()
            plt.show()
