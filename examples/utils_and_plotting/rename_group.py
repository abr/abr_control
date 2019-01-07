from abr_control.utils import DataHandler

dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')
loc = '/friction_post_tuning'
keys = dat.get_keys(loc)
#print(keys)

# for key in keys:
#     if key != 'autogen_targets':
#         if key != 'datestamp':
#             if key != 'timestamp':
#                 print('---------------------')
#                 print('key :', key)
#                 dat.save(data={'test2': [1,2]}, save_location='%s/%s/testtt2'%(loc, key))
#                 params = dat.get_keys(loc + '/' + key)
#                 for param in params:
#                     if 'session' not in param:
#                         print('param: ', param)
#                         place='%s/%s/%s'%(loc,key,param)
#                         new_place = '%s/%s/parameters/%s'%(loc,key,param)
#                         print('OLD: %s \nNEW: %s' %
#                                 (place, new_place))
#                         dat.rename(old=place, new=new_place, delete_old=True)
#dat.delete(loc+'/nengo_cpu_friction_22_0/session004')
for ii in range(14,15):
    print('%03d'%ii)
    dat.rename(old=loc+'/nengo_cpu_friction_21_0/session%03d'%ii,
            new=loc+'/nengo_cpu_friction_22_0/session%03d'%ii, delete_old=True)
