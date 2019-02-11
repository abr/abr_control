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
            use_spherical=False,
            encoders=None, intercepts=None, n_inputs=2, n_outputs=2,
            n_ensembles=1, n_neurons=100):
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

        #TODO: currently hacked in if using loihi where dynadapt is not saved,
        # need to fix on this end or the test saving end
        try:
            dat = DataHandler(use_cache=use_cache, db_name=db_name)
            loc = '/%s/%s/'%(test_group, test_name)
            param_loc = '%sparameters/dynamics_adaptation'%loc
            keys = dat.get_keys(group_path=param_loc)
            adapt_params = dat.load(params=keys, save_location = param_loc)
        except:
            print('Test does not contain "dynamics_adaptation" data...')
            print('Using the following defaults:')
            adapt_params = {
                            'n_input': n_inputs,
                            'n_output': n_outputs,
                            'n_neurons': n_neurons,
                            'n_ensembles': n_ensembles,
                            'pes': 1e-6,
                            'backend': 'nengo_cpu',
                            'probe_weights': True,
                            'seed': 0,
                            'neuron_type': 'lif'
                           }
        print(adapt_params)

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
                intercepts_bounds=None,
                intercepts_mode=None,
                intercepts=intercepts,
                #weights_file=weights,
                backend=adapt_params['backend'],
                probe_weights=adapt_params['probe_weights'],
                seed=int(adapt_params['seed']),
                #neuron_type=np.array2string(adapt_params['neuron_type']))
                neuron_type=adapt_params['neuron_type'],
                encoders=encoders)

    def plot_activity(self, input_signal, time=None, save_num=0,
            getting_ideal_intercepts=False, plot_all_ens=False,
            # TODO: delete this line when done with testing
            error=None, q=None, dq=None, q_baseline=None, dq_baseline=None,
            twinplot=False):
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

        if time is None:
            time = np.ones(len(input_signal))

        if self.use_spherical:
            # convert to spherical
            #input_signal = self.convert_to_spherical(input_signal)
            tmpp = []
            for oo, inputs in enumerate(input_signal):
                if oo % 100 == 0 or oo == len(input_signal):
                    print('%i of %i' %(oo, len(input_signal)))
                tmpp.append(self.convert_to_spherical(inputs))
            input_signal = tmpp
            input_signal = np.array(input_signal)
            #input_signal = np.array(input_signal).T

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
            plt.title('Spiking activity %i\n' %
                    (save_num))
            if twinplot:
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
        plt.hist(time_active, bins=np.linspace(0,1,100))
        ax2.set_title('Proportion of time neurons are active %i\n' %
                (save_num))
        ax2.set_ylabel('Number of active neurons')
        ax2.set_xlabel('Proportion of Time')

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
        ax2.set_title(('Proportion of active neurons over time %i\n'%save_num
            + 'Active: %i,  Inactive: %i'%(num_active, num_inactive)))
        ax2.set_ylabel('Proportion Active')
        ax2.set_xlabel('Time [sec]')
        ax2.set_ylim(0, 1)
        if twinplot:
            if error is not None:
                ax22 = ax2.twinx()
                ax22.set_ylabel('Error [m]')
                ax22.plot(np.cumsum(time), error,'r', label='Error')
                ax2.legend(loc=1)
                ax22.legend(loc=2)
        #print("TIME: ", np.sum(time))
        plt.tight_layout()
        print('Saving Figure to %s/learning_profile_%05d.png'%(self.save_loc, save_num))
        plt.savefig('%s/learning_profile_%05d.png'%(self.save_loc, save_num))
        #plt.savefig('figures/gif_figs/activity/proportion_active_%05d.pdf'%save_num)
        plt.close()

    def convert_to_spherical(self, input_signal):
        #print('Converting input to spherical coordinates...')
        x = input_signal
        pi = np.pi
        spherical = []

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
            # if ss%100 == 0:
            #     print('%i of %i' %(ss, len(x)))
            sphr = sin_product(input_signal=x_rad, count=ss)
            sphr*= np.cos(x_rad[ss])
            spherical.append(sphr)
        spherical.append(sin_product(input_signal=x_2rad, count=len(x)))
        #print('Conversion complete.')
        return(spherical)
