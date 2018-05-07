from abr_control.utils import DataHandler

session=0
run=0
test_name='testing_handler2'
test_group='testing_saving'

print('creating data handler')
dat = DataHandler(use_cache=True)
print('saving data...')
dat.save_data(tracked_data={'test1': [1,1,1], 'test2': [2, 2, 2, 2]},
              session=session,
              run=run,
              test_name=test_name,
              test_group=test_group,
              overwrite=True,
              create=True)

run='run0'
session='session0'
print('getting keys...')
keys = dat.get_keys('%s/%s/%s/%s'%(test_group,test_name,session,run))
print('the keys are: ', keys)

session = 0
run = None
print('loading saved data...')
saved_dat = dat.load_data(params=keys, session=session, run=run, test_name=test_name,
        test_group=test_group)

print('the saved data is: ', saved_dat)
