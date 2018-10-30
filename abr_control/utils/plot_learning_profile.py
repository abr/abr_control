"""
Creates a network to run through a simulation with the provided inputs to get
the neural activity. Three plots will be generated...

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

from abr_control.utils import DataHandler
from abr_control.controllers import signals
from abr_control.utils.paths import cache_dir
import nengo
import nengolib
from nengo.utils.matplotlib import rasterplot

class PlotLearningProfile:
    def __init__(self, test_group, test_name, db_name=None, use_cache=True,
            intercepts_bounds=None, intercepts_mode=None, use_spherical=False):
        """
        Creates the nengo simulation with the same parameters as the test
        passed in

        PARAMETERS
        ----------
        test_group: string
            group path that the desired test belongs to, can include sub groups
            EX: my_awesome_paper/test_type_A
        test_name: string
            the test that data will be loaded from
        db_name: name of the database to load data from
        use_cache: Boolean, Optional (Default: None)
            whether or not the database is saved in the ~/.cache/abr_control
            folder
        intercepts_bounds: list with two floats, Optional (Default: None)
            intercept bounds for the network to see how the activity would
            change with intercept changes. This allows you to quickly test out
            options in simulation with your recorded inputs, without having to
            run a physical arm
        intercepts_mode: float, Optional (Default: None)
            Mode where to distribute intercepts around. Allows you to test out
            alternative in the same way as the intercepts_bounds mentioned
            above
        use_spherical: Boolean, Optional (Default: None)
            If True, increasing the input dimension by 1 and converts inputs to
            spherical coordinates
            NOTE! if the input being passed in was already converted to
            spherical coordinates, set this to False
        """

        # set up save location
        if use_cache:
            self.save_loc = '%s/figures/gif_fig_cache'%(cache_dir)
        else:
            self.save_loc = 'figures/gif_fig_cache'

        files = [f for f in os.listdir(self.save_loc) if f.endswith(".png") ]
        for ii, f in enumerate(files):
            if ii == 0:
                print('Deleting old temporary figures for gif creation...')
            os.remove(os.path.join(self.save_loc, f))
            #print(os.path.join(self.save_loc, f))

        if not os.path.exists(self.save_loc):
            os.makedirs(self.save_loc)

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

        self.use_spherical = use_spherical

        if self.use_spherical:
            extra_dim = True
        else:
            extra_dim = False

        # Create our adaptive ensemble
        self.adapt = signals.DynamicsAdaptation(
                n_input=int(adapt_params['n_input']) + extra_dim,
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

    def plot_activity(self, input_signal, time, save_num=0,
            getting_ideal_intercepts=False, plot_all_ens=False,
            # TODO: delete this line when done with testing
            error=None, q=None, dq=None, q_baseline=None, dq_baseline=None):
        """
        Plots 3 subplots of neural activity to speed up parameter tuning

        PARAMETERS
        ----------
        input_signal: list of floats shape of n_inputs x n_timesteps
        time: list of floats
            timesteps corresponding to the input signal
        save_num: int, Optional (Default: 0)
            number to append to filename of image that is saved
        get_ideal_intercepts: Boolean, Optional (Default: False)
            if using script to run through all possible intercept options to
            find the ideal set, setting this to True skips over some
            unnecessary sections to speed up the process
        plot_all_ens: Boolean, Optional (Default: False)
            True to show all ensembles on rasterplot
            False to show the first dimension
            When using large populations of neurons (Ex:1k neurons x 20
            ensembles) it can slow down plotting of the rasterplot
        """

        if self.use_spherical:
            # convert to spherical
            input_signal = np.array(input_signal).T
            input_signal = self.convert_to_spherical(input_signal)
            input_signal = np.array(input_signal).T

        if not getting_ideal_intercepts:
            # create probes to get rasterplot
            fig = plt.figure(figsize=(8,15))
            print('Creating spike probes...')
            with self.adapt.nengo_model:
               if not hasattr(self.adapt.nengo_model, 'ens_probes'):
                   self.adapt.nengo_model.ens_probes = []
                   for ens in self.adapt.adapt_ens:
                       self.adapt.nengo_model.ens_probes.append(nengo.Probe(ens.neurons,
                               synapse=None))

            sim = nengo.Simulator(self.adapt.nengo_model)
            temp_in = input_signal
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
                if not plot_all_ens:
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
            if q is not None and dq is not None:
                ax2.plot(np.cumsum(time), q[0], 'r', label='q1 adaptive')
                ax2.plot(np.cumsum(time), dq[0], 'b', label='dq1 adaptive')
                ax2.plot(np.cumsum(time), q[1], 'g', label='q2 adaptive')
                ax2.plot(np.cumsum(time), dq[1], 'c', label='dq2 adaptive')
                ax2.plot(np.linspace(time[0], sum(time), len(q_baseline[0])),
                        q_baseline[0], 'r--', label='q1 baseline')
                ax2.plot(np.linspace(time[0], sum(time), len(dq_baseline[0])),
                        dq_baseline[0], 'b--', label='dq1 baseline')
                ax2.plot(np.linspace(time[0], sum(time), len(q_baseline[1])),
                        q_baseline[1], 'g--', label='q2 baseline')
                ax2.plot(np.linspace(time[0], sum(time), len(dq_baseline[1])),
                        dq_baseline[1], 'c--', label='dq2 baseline')
                ax2.legend()
            else:
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

        print('Plotting proportion of active time...')
        time_active = np.hstack(time_active)
        if getting_ideal_intercepts:
            return (time_active, activities)
        ax2 = fig.add_subplot(312)
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
        ax2.plot(np.cumsum(time), proportion_active, label='proportion active')
        ax2.set_title('Proportion of active neurons over time %i'%save_num)
        ax2.set_ylabel('Proportion Active')
        ax2.set_xlabel('Time [sec]')
        ax2.set_ylim(0, 1)
        if error is not None:
            ax22 = ax2.twinx()
            ax22.set_ylabel('Error [m]')
            ax22.plot(np.cumsum(time), error,'r', label='Error')
            ax2.legend(loc=1)
            ax22.legend(loc=2)
        #print("TIME: ", np.sum(time))
        plt.tight_layout()
        print('Saving Figure')
        plt.savefig('%s/learning_profile_%05d.png'%(self.save_loc, save_num))
        #plt.savefig('figures/gif_figs/activity/proportion_active_%05d.pdf'%save_num)
        plt.close()

    def convert_to_spherical(self, input_signal):
        """
        Convert the inputs to spherical coordinate, this will increase the
        number of input dimensions by 1

        PARAMETERS
        ----------
        input_signal: list of floats shape of n_inputs x n_timesteps
        """
        x0 = input_signal[0]
        x1 = input_signal[1]
        x2 = input_signal[2]
        x3 = input_signal[3]

        spherical = [
                     np.cos(x0),
                     np.sin(x0)*np.cos(x1),
                     np.sin(x0)*np.sin(x1)*np.cos(x2),
                     np.sin(x0)*np.sin(x1)*np.sin(x2)*np.cos(x3),
                     np.sin(x0)*np.sin(x1)*np.sin(x2)*np.sin(x3)
                    ]
        #print(spherical)
        mag = np.linalg.norm(spherical, axis=0)
        #print('Magnitue of spherical input: ', mag)
        return spherical
