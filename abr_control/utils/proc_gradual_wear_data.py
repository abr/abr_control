from abr_control.utils import PathErrorToIdeal
from abr_control.utils import PlotError

proc = PathErrorToIdeal()
plt = PlotError()

test_group = 'loihi2018/weighted'
test_list = ['pd/not_weighted_19',
              'pd/weighted_18',
              'pid/weighted_29',
              'nengo/1000_neurons_x_1/weighted_46',
              'nengo_ocl/1000_neurons_x_1/weighted_34'
              ]
# test_group = 'loihi2018/no_weight'
# test_list = ['pd/pd_5pt_baseline',
#              'pid/gradual_wear14',
#              'nengo/1000_neurons_x_20/gradual_wear23',
#              'nengo_ocl/1000_neurons_x_20/gradual_wear21',
#              'chip/gradual_wear24'
#              ]
# Get Positional Error
proc.process(test_group=test_group,
             test_list=test_list,
             regenerate=True,
             use_cache=True,
             order_of_error=0,
             upper_baseline_loc=test_list[1],
             lower_baseline_loc=test_list[0])

# Get Velocity Error
proc.process(test_group=test_group,
             test_list=test_list,
             regenerate=True,
             use_cache=True,
             order_of_error=1,
             upper_baseline_loc=test_list[1],
             lower_baseline_loc=test_list[0])

plt.get_error_plot(test_group=test_group,
                   test_list=test_list,
                   show_plot=True,
                   save_figure=False,
                   order_of_error=[0, 1])
