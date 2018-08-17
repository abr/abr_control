from abr_control.utils import plot_torque_profile

plot_torque_profile.plot_data(test_group='1lb_random_target',
                                tests = [
                                         # 'nengo_cpu_57_9',
                                         # 'pd_no_weight_9',
                                         # 'pd_no_weight_11',
                                         'pd_no_weight_14',
                                        ],
                                session=0,
                                #runs = [0,1,2,3,4,5,6,7,8,9],
                                runs = [1],
                                use_cache=True,
                                db_name='dewolf2018neuromorphic')
