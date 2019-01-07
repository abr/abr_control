from abr_control.utils import PathErrorToIdeal
from abr_control.utils import PlotError

proc = PathErrorToIdeal()
plt = PlotError()
db_name='dewolf2018neuromorphic'
regen = False
plot_only = False
orders_of_error = [
                   0,
                   # 1,
                   # 3
                   ]
title = [
         'abs-pos-error',
         # 'abs-vel-error',
         # 'abs-jerk-error'
        ]
y_label = [
           'm*s',
           # 'm',
           # 'm/s^2'
           ]

test_group = 'friction_post_tuning'
test_list = [
              'pd_no_friction_5_0',
              'pd_friction_11_0',
              #
              #
              # 1k
              # 'nengo_cpu_friction_22_0',
              # # 5k
              # 'nengo_cpu_friction_21_0',
              # # 10k
              # 'nengo_cpu_friction_20_0',
              # 20k cpu
              'nengo_cpu_friction_12_0',
              # # 50k cpu
              # 'nengo_cpu_friction_19_0',
              # # 20k cpu non spherical
              'nengo_cpu_friction_23_0',

              # 1k
              # 'nengo_gpu_friction_5_0',
              # # 5k
              # 'nengo_gpu_friction_4_0',
              # # 10k
              # 'nengo_gpu_friction_2_0',
              # # 20k
              # 'nengo_gpu_friction_1_0',
              # # 50k
              # 'nengo_gpu_friction_3_0',


              # ## 50k SLI1 SLF16
              # 'nengo_loihi_friction_9_0',
              # # ## 20k SLI1 SLF16
              # 'nengo_loihi_friction_10_0',
              # # ## 100k SLI1 SLF16
              #'nengo_loihi_friction_11_0',
              # # # ## 100 SLI1 SLF16
              # #'nengo_loihi_friction_12_0',
              # 100 neurons no weight saving
              # 'nengo_loihi_friction_13_0',
              # 1 neurons no weight saving
              # 'nengo_loihi_friction_14_0',
              # # 10 neurons no weight saving
              # 'nengo_loihi_friction_15_0',
              # # 1 neuron SLF16 SLI1 with learning
              # 'nengo_loihi_friction_16_0',
              # #10 neuron SLF16 SLI1 with learning
              # 'nengo_loihi_friction_17_0',
              ]
labels = ['','',': 20k spherical', ': 20k non spherical', ': 10k', ': 20k', ': 50k']
# labels = ['', '', ': 1 neuron no weight saving',
#           ': 10 neurons no weight saving', ': 1 neuron',
#           ': 10 neurons']
legend_labels = []
for ii, test in enumerate(test_list):
    legend_labels.append(test + labels[ii])
print(legend_labels)

if not plot_only:
    print('Processing Data...')
    for order in orders_of_error:
        proc.process(test_group=test_group,
                     test_list=test_list,
                     regenerate=regen,
                     use_cache=True,
                     order_of_error=order,
                     # upper_baseline_loc=test_list[1],
                     # lower_baseline_loc=test_list[0],
                     db_name=db_name,
                     path_planner_as_ideal=True,
                     n_sessions=[15, 15, 3,3,15, 15, 15, 15],
                     n_runs=50)
        #TODO: if have incomplete session and run is not
        # specified, the next module will take the smallest
        # set of runs and use that for each session, should
        # check if incompelte and use the previous sessions
        # number of runs

print('Plotting Data...')
for ii, entry in enumerate(title):
    plt.get_error_plot(test_group=test_group,
                       test_list=test_list,
                       show_plot=False,
                       save_figure=True,
                       use_cache=True,
                       db_name=db_name,
                       order_of_error=[orders_of_error[ii]],
                       sum_errors=False,
                       scaling_factor=1,
                       colors=['k', 'b', 'g', 'r', 'y', 'm', 'tab:grey'],
                       y_label=y_label[ii],
                       fig_title=entry,
                       clear_plot=True,
                       legend_loc=0,
                       legend_labels=legend_labels)
