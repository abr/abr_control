from abr_control.utils import PathErrorToIdeal
from abr_control.utils import PlotError

proc = PathErrorToIdeal()
plt = PlotError()
db_name='dewolf2018neuromorphic'
orders_of_error = [0,3]
title = [
        'abs-pos-error-wear',
        'abs-jerk-error-wear'
        ]
y_label = ['m*s', 'm/s^2']

test_group = 'friction_tests'
test_list = [
              'pd_no_friction',
              'pd',
              'pid',
              'nengo_cpu1k',
              'nengo_gpu1k',
              'nengo_loihi1k'
              ]

for order in orders_of_error:
    proc.process(test_group=test_group,
                 test_list=test_list,
                 regenerate=True,
                 use_cache=True,
                 order_of_error=order,
                 # upper_baseline_loc=test_list[1],
                 # lower_baseline_loc=test_list[0],
                 db_name=db_name,
                 n_runs=50)

for ii, entry in enumerate(title):
    plt.get_error_plot(test_group=test_group,
                       test_list=test_list,
                       show_plot=False,
                       save_figure=True,
                       use_cache=True,
                       db_name=db_name,
                       order_of_error=[orders_of_error[ii]],
                       sum_errors=True,
                       scaling_factor=100,
                       colors=['k', 'b', 'g', 'r', 'm', 'y'],
                       y_label=y_label[ii],
                       fig_title=entry,
                       clear_plot=True)
