import os
import numpy as np

from abr_control.utils import DataHandler
from abr_control.utils.paths import cache_dir

class ConvertData():
    def __init__(self, use_cache=True, db_name=None):
        self.dat = DataHandler(use_cache=use_cache, db_name=db_name)
        self.use_cache = use_cache

    def track_unsaved(self, root, name, data):
        # print('error saving %s/%s'%(root,name))
        dataset = {name: data}
        self.dat.save(data=dataset,
                save_location='UNSAVED_DATA%s'%root.replace(cache_dir,''),
                overwrite=False, create=True)

    def is_run_or_session(self, root):
        root = root.split('/')
        # print('split root is ', root)
        try:
            for ii, s in enumerate(root):
                if 'run' in s:
                #if 'run' in root[-1]:
                    # print('Folder is a run folder')
                    new_group_num = int(root[ii].split('run')[1].split('_')[0])
                    # print('new group num %i'%new_group_num)
                    new_group = 'run%03d'%new_group_num
                    # print('new group %s'%new_group)
                    root[ii] = new_group

                if 'session' in s:
                #elif 'session' in root[-1]:
                    # print('Folder is a session folder')
                    new_group_num = int(root[ii].split('session')[1])
                    # print('new group num %i'%new_group_num)
                    new_group = 'session%03d'%new_group_num
                    # print('new group %s'%new_group)
                    root[ii] = new_group
        except Exception:
            # if the string doesn't follow the format then save it as is
            pass

        group_name = '/'.join(root)

        return group_name

    def convert_data(self, old_location, new_location=None, notes=None):
        if new_location is None:
            new_location = old_location

        for ii, loc in enumerate(old_location):
            if self.use_cache:
                search_dir = cache_dir + loc
            else:
                search_dir = loc

            # print('searching: ', search_dir)

            for root, dirs, files in os.walk(search_dir):
                # check if the current folder is a run or session folder, in which case we
                # want to convert the last number to a %03d
                new_root = self.is_run_or_session(root)
                # check if the current root folder exists as a group in the db, if not create it/
                self.dat.check_group_exists(location=new_root.replace(cache_dir,''), create=True)
                if files:
                    for name in files:
                        if name[-4:] == '.npz':
                            # print('loading %s'%name)
                            keys = np.load(root+'/'+name).keys()
                            for key in keys:
                                # if key == 'error':
                                #     print('key is: ', key)
                                loaded_dat = np.ndarray.tolist(
                                        np.squeeze(np.load(root+'/'+name)[key]))
                                # if key == 'error':
                                #     print('data is: ', loaded_dat)
                                d = {key: loaded_dat}
                                # if key == 'error':
                                #     print('dict format is: \n\n\n\n\n\n\n\n\n\n\n', d)
                                try:
                                    if new_location is None:
                                        save_loc = new_root.replace(cache_dir,'')
                                    else:
                                        save_loc = ('%s%s')%(new_location[ii],
                                            new_root.replace(search_dir,''))

                                    self.dat.save(data=d, save_location=save_loc,
                                            overwrite=True,
                                            create=True)

                                    if notes is not None:
                                        self.dat.save(data=notes[ii],
                                                save_location=save_loc,
                                                overwrite=True,
                                                create=True)

                                except Exception as error:
                                    # if data doesn't save, track it
                                    self.track_unsaved(root,name, data=str(error))

                        else:
                            self.track_unsaved(root, name, data='NOT AN NPZ FILE')

