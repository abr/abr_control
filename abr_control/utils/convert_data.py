import os
import numpy as np

from abr_control.utils import DataHandler
from abr_control.utils.paths import cache_dir

class ConvertData():
    """
    Convert data from npz files in folders to hdf5 database with
    groups and datasets
    """
    def __init__(self, use_cache=True, db_name=None):
        """
        Connect to database, create it if the provided name does not exist

        PARAMETERS
        ----------
        use_cache = Boolean, Optional (Default: True)
            True: if database is in ~/.cache/abr_control folder, or if it is desired
            for a newly created database to be saved there.
            False: database is or will be saved to working directory
        db_name: String, Optional (Default: None)
            The name of the database being loaded or created
            If None, then the default database name will be used
        """
        self.dat = DataHandler(use_cache=use_cache, db_name=db_name)
        self.use_cache = use_cache

    def track_unsaved(self, root, name, data):
        """
        Saves path to data that was not successfully converted to hdf5 group at
        root of database called UNSAVED_DATA
        *NOTE* this data IS NOT saved in the hdf5 database so do not delete it
        if you still need it. The database can be viewed with hdf5's viewer
        software, or the abr_control gui, to see what files were not
        successfully converted and saved

        PARAMETERS
        ----------
        root: String
            The path to the current group
        name: String
            Name of the current group
        data: String or list
            Data to be saved to a dataset in the 'name' group passed in
            Default is to save the error that appeared when trying to
            convert/save
        """
        # print('error saving %s/%s'%(root,name))
        dataset = {name: data}
        self.dat.save(data=dataset,
                save_location='UNSAVED_DATA%s'%root.replace(cache_dir,''),
                overwrite=False, create=True)

    def is_run_or_session(self, root):
        """
        Checks if the provided path points to a session or run folder.

        The main purpose of this is to convert the run and session numbers to
        %03d format in the database

        PARAMETERS
        ----------
        root: string
            path to the current folder

        """
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
        """
        Converts npz data in old_location and saves it to hdf5 database in
        new_location group

        PARAMETERS
        ----------
        old_location: string
            path to npz data to be converted to hdf5
        new_location: string, Optional (Default: None)
            group to save new data to in hdf5 database, can be path with
            multiple groups
            EX: maingroup/sub_group/type_a_data
            If left as None, the same folder structure in 'old_location' will
            be used
        notes: string, Optional (Default: None)
            any notes to be saved with database, such as original path or test
            notes
        """
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

