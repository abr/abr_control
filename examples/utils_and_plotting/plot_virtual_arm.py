from abr_control.utils import DataHandler
"""
"""

db = 'dewolf2018neuromorphic'

test_groups = [
                'friction_post_tuning',
                'friction_post_tuning',
              ]
tests = [
        # 'nengo_loihi_friction_2_0',
        # 'nengo_loihi_friction_1_0',
        #'nengo_cpu_friction_5_0',
        #'pd_friction_7_0',
        'pd_no_friction_5_0',
        'pd_no_friction_3_0',
        #'nengo_cpu_weight_17_99',
        #'pd_weight_9',
        #'pd_weight_8',
        # 'pd_no_weight_14',
        # 'pd_no_weight_7',
        ]

dat = DataHandler(use_cache=True, db_name=source_db)
