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
    def __init__(self, test_group, test_name, db_name=None, use_cache=True):

        dat = DataHandler(use_cache=use_cache, db_name=db_name)

        loc = '/%s/%s/'%(test_group, test_name)
        param_loc = '%sparameters/dynamics_adaptation'%loc
        keys = dat.get_keys(group_path=param_loc)
        adapt_params = dat.load(params=keys, save_location = param_loc)
        # run_data = dat.load(params=['input_signal', 'time'],
        #         save_location='%ssession%03d/run%03d'
        #         %(loc, session, run))
        # self.input_signal = run_data['input_signal']
        # self.time = run_data['time']

        # if run == 0:
        #     weights = None
        # else:
        #     weights = dat.load(params=['weights'],
        #         save_location='%ssession%03d/run%03d'
        #         %(loc, session, run-1))

        # Create our adaptive ensemble
        self.adapt = signals.DynamicsAdaptation(
                n_input=int(adapt_params['n_input']),
                n_output=int(adapt_params['n_output']),
                n_neurons=int(adapt_params['n_neurons']),
                n_ensembles=int(adapt_params['n_ensembles']),
                pes_learning_rate=float(adapt_params['pes']),
                intercepts=adapt_params['intercepts_bounds'],
                intercepts_mode=adapt_params['intercepts_mode'],
                #weights_file=weights,
                backend=adapt_params['backend'],
                probe_weights=adapt_params['probe_weights'],
                seed=int(adapt_params['seed']),
                neuron_type=np.array2string(adapt_params['neuron_type']))

    def plot_activity(self, input_signal, time, error, q, dq, input_joints, save_num=0):

        with self.adapt.nengo_model:
           if not hasattr(self.adapt.nengo_model, 'ens_probes'):
               self.adapt.nengo_model.ens_probes = []
               for ens in self.adapt.adapt_ens:
                   self.adapt.nengo_model.ens_probes.append(nengo.Probe(ens.neurons,
                           synapse=None))

        sim = nengo.Simulator(self.adapt.nengo_model)
        for inputs in input_signal:
            self.adapt.input_signal = inputs
            sim.run(time_in_seconds=0.001, progress_bar=False)

        fig = plt.figure(figsize=(8,15))
        #n_plts = len(self.adapt.nengo_model.ens_probes) + 2 + 1
        #plt_count = 1
        for probe in self.adapt.nengo_model.ens_probes:
            ax = fig.add_subplot(3,1,1)
            a = rasterplot(np.cumsum(time),sim.data[probe], ax=ax)
            #a = rasterplot(sim.trange(),sim.data[probe], ax=ax)
            # plt_count += 1
            #ax = rasterplot(srm.trange(),sim.data[probe])
            plt.ylabel('Neuron')
            plt.title('Spiking activity %i' %save_num)
            # plt.savefig('figures/gif_figs/rasterplot/rasterplot_%05d.png'%save_num)
            # plt.close()

        ax = fig.add_subplot(3,1,2)
        ax2 = ax.twinx()
        ax.set_title('joint inputs')
        ax.set_ylabel('Joint Angle [rad]')
        ax2.set_ylabel('Joint Velocity [rad/sec]')
        for ii, using_joint in enumerate(input_joints):
            if using_joint:
                plt.plot(q[:,ii], label='q%i'%ii)
                ax2.plot(dq[:,ii], label='dq%i'%ii)

        plt.legend(loc=1)

        activities = []
        for ens in self.adapt.adapt_ens:
            _, activity = nengo.utils.ensemble.tuning_curves(ens,
                    self.adapt.sim, input_signal)
            activity[activity>1e-5]=1
            activity[activity<=1e-5]=0
            activities.append(np.copy(activity))

        proportion_active = []
        for activity in activities:
            # axis=0 mean over time
            # axis=1 mean over neurons
            # len(activity.T) gives the number of neurons
            proportion_active.append(np.sum(activity, axis=1)/len(activity.T))

        proportion_active = np.sum(proportion_active,
                axis=0)/len(proportion_active)
        #print('prop: ', proportion_active.shape)
        #seaborn.distplot(proportion_active)
        # plt.figure()
        # plt.plot(np.cumsum(time), proportion_active)
        ax2 = fig.add_subplot(313)
        ax2.plot(np.cumsum(time), proportion_active)
        ax2.set_title('proportion of active neurons %i'%save_num)
        ax2.set_ylabel('Proportion Active')
        ax2.set_xlabel('Time [sec]')
        ax2.set_ylim(0, 1)
        ax2_err = ax2.twinx()
        ax2_err.plot(np.cumsum(time), error, 'r')
        ax2_err.set_ylabel('Error to Target [m]')
        ax2_err.set_ylim(0, 1)
        print("TIME: ", np.sum(time))
        plt.tight_layout()
        plt.savefig('figures/gif_figs/activity/proportion_active_%05d.png'%save_num)
        plt.close()


        # plt.figure()
        # plt.plot(np.arange(activity.shape[0])*np.cumsum(time)/1000, np.mean(activity>0, axis=1))
        # plt.title('proportion of neurons active over time %i' %save_num)
        # plt.ylim(0,1)
        # plt.savefig('figures/gif_figs/activity_time/proportion_active_time_%05d.png'%save_num)
        # plt.close()

if __name__ == '__main__':
    test_group = '1lb_random_target'
    test_name = 'nengo_cpu_16_8'
    db_name = 'dewolf2018neuromorphic'
    loc = '/%s/%s/'%(test_group, test_name)
    use_cache = True
    n_runs = 10

    plt_learning = PlotLearningProfile(test_group=test_group,
            test_name=test_name, db_name=db_name, use_cache=use_cache)

    dat = DataHandler(use_cache=use_cache, db_name=db_name)
    session = 0
    for run in range(0, n_runs):
        run_data = dat.load(params=['input_signal', 'time', 'error', 'q', 'dq'],
                save_location='%ssession%03d/run%03d'
                %(loc, session, run))
        training_data = dat.load(params=['adapt_input'],
                save_location='%s/parameters/training' %loc)
        adapt_input = training_data['adapt_input']
        input_signal = run_data['input_signal']
        time = run_data['time']
        error = run_data['error']
        q = run_data['q']
        dq = run_data['dq']

        plt_learning.plot_activity(input_signal=input_signal, time=time,
                error=error, save_num=run, q=q, dq=dq, input_joints=adapt_input)


    def make_gif(fig_loc, save_loc, save_name, delay=5):
        import subprocess
        bashCommand = ("convert -delay %i -loop 0 -deconstruct -quantize"%delay
                       + " transparent -layers optimize -resize 1200x2000"
                       + " %s/*.png %s/%s.gif"
                       %(fig_loc, save_loc, save_name))
        print('100.00% complete')
        print('Creating gif...')
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        print('Finished')

    figs = 'figures/gif_figs'
    names = ['rasterplot', 'activity']
    for name in names:
        make_gif(fig_loc='%s/%s'%(figs, name),
                 save_loc='%s/%s'%(figs, name),
                 save_name=name,
                 delay=200)
