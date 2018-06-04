import os
import numpy as np

from abr_control.utils import DataHandler
from abr_control.utils.paths import cache_dir

dat = DataHandler(use_cache=True)
search_dir = cache_dir + '/paper1'

for root, dirs, files in os.walk(search_dir):
	# check if the current root folder exists as a group in the db, if not create it
	dat.check_group_exists(location=root.replace(cache_dir,''), create=True)
	if files:
		for name in files:
			print('loading %s'%name)
			a = np.load(root+'/'+name)
			d = dict(zip(('{}'.format(k) for k in a), (a[k] for k in a)))
			dat.save(data=d, save_location=root.replace(cache_dir,''), overwrite=True, create=True)
	
