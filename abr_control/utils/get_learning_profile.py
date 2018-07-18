"""
Runs a simulation using the data recorded from a run of learning to get
information about the neural network
Plots will be created for each reach, or the inputs can be stacked to get
activity over the whole reaching space

The intercepts bounds and mode can be altered to see how it would affect the
network provided the same input. This helps tune your network without having to
rerun tests

Plots
1. rasterplot showing spikes for each neuron over time
2. proportion of time active, the number of neurons active for what proportion
   of run time
3. proportion of neurons that are active over time
"""
from abr_control.utils import DataHandler, PlotLearningProfile
import numpy as np
import time as t
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import time as t
s = t.time()
test_group = '1lb_random_target'
test_name = 'nengo_cpu_20_0'
db_name = 'dewolf2018neuromorphic'
loc = '/%s/%s/'%(test_group, test_name)
use_cache = True
n_runs = 10

plt_learning = PlotLearningProfile(test_group=test_group,
        test_name=test_name, db_name=db_name, use_cache=use_cache,
        intercepts_bounds=None, intercepts_mode=None)

dat = DataHandler(use_cache=use_cache, db_name=db_name)
# pass input signals for target reaches individually, creates gif of
# activity for each individual reach
session = 0
for run in range(0, n_runs):
    run_data = dat.load(params=['input_signal', 'time'],
            save_location='%ssession%03d/run%03d'
            %(loc, session, run))
    training_data = dat.load(params=['adapt_input'],
            save_location='%s/parameters/training' %loc)
    adapt_input = training_data['adapt_input']
    input_signal = run_data['input_signal']
    time = run_data['time']
    plt_learning.plot_activity(input_signal=input_signal, time=time,
            save_num=run, input_joints=adapt_input)

# stack input signal to show activity over the enitre reaching space
# adapt_input = []
# input_signal = []
# time = []
# for run in range(0, n_runs):
#     run_data = dat.load(params=['input_signal', 'time', 'error', 'q', 'dq'],
#             save_location='%ssession%03d/run%03d'
#             %(loc, session, run))
#     training_data = dat.load(params=['adapt_input'],
#             save_location='%s/parameters/training' %loc)
#     adapt_input.append(training_data['adapt_input'])
#     input_signal.append(run_data['input_signal'])
#     time.append(run_data['time'])
#     error = run_data['error']
#     q = run_data['q']
#     dq = run_data['dq']
#
# input_signal = np.vstack(input_signal)
# time = np.hstack(time)
# plt_learning.plot_activity(input_signal=input_signal, time=time,
#         error=error, save_num=run, q=q, dq=dq, input_joints=adapt_input)


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
names = ['activity']
print('Creating gifs')
for name in names:
    make_gif(fig_loc='%s/%s'%(figs, name),
             save_loc='%s/%s'%(figs, name),
             save_name=name,
             delay=200)
print(t.time()-s)
