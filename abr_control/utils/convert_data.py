import os
import numpy as np

from abr_control.utils import DataHandler
from abr_control.utils.paths import cache_dir

def track_unsaved(root, name, data):
    print('error saving %s/%s'%(root,name))
    unsaved_files[(root.replace(cache_dir,'')
            +'__'+name)] = data

dat = DataHandler(use_cache=True)
loc = '/loihi2018'
cache_dir += '/saved_weights'
search_dir = cache_dir + loc
unsaved_files = {}
for root, dirs, files in os.walk(search_dir):
	# check if the current root folder exists as a group in the db, if not create it
	dat.check_group_exists(location=root.replace(cache_dir,''), create=True)
	if files:
		for name in files:
                        if name[-4:] == '.npz':
                            print('loading %s'%name)
                            a = np.load(root+'/'+name)
                            d = dict(zip(('{}'.format(k) for k in a), (a[k] for k in a)))
                            try:
                                dat.save(data=d, save_location=root.replace(cache_dir,''), overwrite=True, create=True)
                            except Exception as error:
                                # if data doesn't save, track it
                                track_unsaved(root,name, data=str(error))

                        else:
                            track_unsaved(root, name, data='NOT AN NPZ FILE')

print('saving link to unsaved data')
dat.save(data=unsaved_files,
        save_location='UNSAVED_DATA',
        overwrite=False, create=True)
