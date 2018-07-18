"""
Creates a network to run through a simulation with the provided inputs to get
the neural activity. Three plots will be generated...

Plots
1. rasterplot showing spikes for each neuron over time
2. proportion of time active, the number of neurons active for what proportion
   of run time
3. proportion of neurons that are active over time
"""
import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
import seaborn
import numpy as np

from abr_control.utils import DataHandler
from abr_control.controllers import signals
import nengo
import nengolib
from nengo.utils.matplotlib import rasterplot

class PlotLearningProfile:
    def __init__(self, test_group, test_name, db_name=None, use_cache=True,
            intercepts_bounds=None, intercepts_mode=None):

        dat = DataHandler(use_cache=use_cache, db_name=db_name)

        loc = '/%s/%s/'%(test_group, test_name)
        param_loc = '%sparameters/dynamics_adaptation'%loc
        keys = dat.get_keys(group_path=param_loc)
        adapt_params = dat.load(params=keys, save_location = param_loc)
        if intercepts_bounds is None:
            self.intercepts_bounds = adapt_params['intercepts_bounds']
        else:
            self.intercepts_bounds = intercepts_bounds
        if intercepts_mode is None:
            self.intercepts_mode = adapt_params['intercepts_mode']
        else:
            self.intercepts_mode = intercepts_mode

        # Create our adaptive ensemble
        self.adapt = signals.DynamicsAdaptation(
                n_input=int(adapt_params['n_input']),
                n_output=int(adapt_params['n_output']),
                n_neurons=int(adapt_params['n_neurons']),
                n_ensembles=int(adapt_params['n_ensembles']),
                pes_learning_rate=float(adapt_params['pes']),
                intercepts=self.intercepts_bounds,
                intercepts_mode=self.intercepts_mode,
                #weights_file=weights,
                backend=adapt_params['backend'],
                probe_weights=adapt_params['probe_weights'],
                seed=int(adapt_params['seed']),
                neuron_type=np.array2string(adapt_params['neuron_type']))

    def plot_activity(self, input_signal, time, input_joints, save_num=0,
            getting_ideal_intercepts=False):

        fig = plt.figure(figsize=(8,15))
        if not getting_ideal_intercepts:
            print('Creating probes')
            with self.adapt.nengo_model:
               if not hasattr(self.adapt.nengo_model, 'ens_probes'):
                   self.adapt.nengo_model.ens_probes = []
                   for ens in self.adapt.adapt_ens:
                       self.adapt.nengo_model.ens_probes.append(nengo.Probe(ens.neurons,
                               synapse=None))

            sim = nengo.Simulator(self.adapt.nengo_model)
            temp_in = input_signal * 0
            print('Running sim...')
            for ii, inputs in enumerate(input_signal):
                #inputs = input_signal[40]
                self.adapt.input_signal = inputs
                sim.run(time_in_seconds=0.001, progress_bar=False)
                temp_in[ii] = inputs
            print('Sim complete')

            probes = []
            print('Plotting spiking activity...')
            for probe in self.adapt.nengo_model.ens_probes:
                probes.append(sim.data[probe])
                #NOTE: uncomment break if you want to show rasterplot for all
                # neurons
                break
            probes = np.hstack(probes)
            ax = fig.add_subplot(3,1,1)
            a = rasterplot(np.cumsum(time),probes, ax=ax)
            plt.ylabel('Neuron')
            plt.xlabel('Time [sec]')
            plt.title('Spiking activity %i\n %s, %s' %
                    (save_num, self.intercepts_bounds,
                        self.intercepts_mode))
            ax2 = ax.twinx()
            ax2.plot(np.cumsum(time), temp_in, 'k')
            ax2.set_ylabel('input signal')

        activities = []
        print('Plotting proportion of active neurons...')
        for ens in self.adapt.adapt_ens:
            _, activity = nengo.utils.ensemble.tuning_curves(ens,
                    self.adapt.sim, input_signal)
            activity[activity>1e-5]=1
            activity[activity<=1e-5]=0
            activities.append(np.copy(activity))

        time_active = []
        for activity in activities:
            # axis=0 mean over time
            # axis=1 mean over neurons
            time_active.append(np.sum(activity, axis=0)/len(activity))

        time_active = np.hstack(time_active)
        ax2 = fig.add_subplot(312)
        if getting_ideal_intercepts:
            return time_active
        plt.hist(time_active, bins=np.linspace(0,1,30))
        ax2.set_title('Proportion of time neurons are active %i\n %s, %s' %
                (save_num, self.intercepts_bounds,
                    self.intercepts_mode))
        ax2.set_ylabel('Number of active neurons')
        ax2.set_xlabel('Proportion of Time')

        proportion_active = []
        for activity in activities:
            # axis=0 mean over time
            # axis=1 mean over neurons
            # len(activity.T) gives the number of neurons
            proportion_active.append(np.sum(activity, axis=1)/len(activity.T))

        proportion_active = np.sum(proportion_active,
                axis=0)/len(proportion_active)
        ax2 = fig.add_subplot(313)
        ax2.plot(np.cumsum(time), proportion_active)
        ax2.set_title('Proportion of active neurons over time %i'%save_num)
        ax2.set_ylabel('Proportion Active')
        ax2.set_xlabel('Time [sec]')
        ax2.set_ylim(0, 1)
        print("TIME: ", np.sum(time))
        plt.tight_layout()
        plt.savefig('figures/gif_figs/activity/proportion_active_%05d.png'%save_num)
        plt.savefig('figures/gif_figs/activity/proportion_active_%05d.pdf'%save_num)
        plt.close()
