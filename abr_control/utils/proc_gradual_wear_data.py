from abr_control.utils import PathErrorToIdeal

proc = PathErrorToIdeal()
test_group = 'loihi2018/no_weight'
test_list = ['pd/pd_5pt_baseline',
             'pid/gradual_wear14',
             'nengo/1000_neurons_x_20/gradual_wear23',
             'nengo_ocl/1000_neurons_x_20/gradual_wear21',
             'chip/gradual_wear24'
             ]
proc.process(test_group=test_group,
             test_list=test_list[2:],
             regenerate=True,
             use_cache=True,
             order_of_error=0,
             upper_baseline_loc=test_list[1],
             lower_baseline_loc=test_list[0])
