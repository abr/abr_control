from abr_control.utils import PathErrorToIdeal
from abr_control.utils import PlotError

proc = PathErrorToIdeal()
plt = PlotError()
db_name='dewolf2018neuromorphic'

test_group = 'gradual_friction_tests'
test_list = [
              'pd_no_friction',
              'pd',
              'pid',
              'nengo_cpu20k',
              'nengo_gpu20k',
              'nengo_loihi20k'
              ]
# Get Positional Error
proc.process(test_group=test_group,
             test_list=test_list,
             regenerate=True,
             use_cache=True,
             order_of_error=0,
             upper_baseline_loc=test_list[1],
             lower_baseline_loc=test_list[0],
             db_name=db_name)

# Get Velocity Error
proc.process(test_group=test_group,
             test_list=test_list,
             regenerate=True,
             use_cache=True,
             order_of_error=1,
             upper_baseline_loc=test_list[1],
             lower_baseline_loc=test_list[0],
             db_name=db_name)

plt.get_error_plot(test_group=test_group,
                   test_list=test_list,
                   show_plot=True,
                   save_figure=False,
                   use_cache=True,
                   db_name=db_name,
                   order_of_error=[0, 1],
                   sum_errors=True,
                   scaling_factor=100)
