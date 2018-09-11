from abr_control.utils import DataHandler
"""
Take a section of a hdf5 database and copy it to a new one.

Useful to send parts of a database instead of a large file
"""

source_db = 'partialdewolf2018'
destination_db = 'dewolf2018neuromorphic'
test_group = 'simulations'
test_list = ['ur5_sim_no_weight_6']

dat1 = DataHandler(use_cache=True, db_name=source_db)
dat2 = DataHandler(use_cache=True, db_name=destination_db)

for test in test_list:
    root_keys = dat1.get_keys('%s/%s'%(test_group,test))
    print('root keys: ', root_keys)
    for group in root_keys:
        sub_keys = dat1.get_keys('%s/%s/%s'%(test_group, test, group))
        print('sub keys: ', sub_keys)
        for sub in sub_keys:
            data_keys = dat1.get_keys('%s/%s/%s/%s'%(test_group, test, group,
                sub))
            data_dict = dat1.load(params=data_keys, save_location='%s/%s/%s/%s'
                    %(test_group, test, group, sub))
            print('sub: ', sub)
            print(data_dict)
            # save to new db
            dat2.save(data=data_dict, save_location='%s/%s/%s/%s'
                    %(test_group, test, group, sub))
