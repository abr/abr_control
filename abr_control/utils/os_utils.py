import os

def makedirs(folder):
    """ Checks to see if folder exists, if it does not, then
    it creates the folder and all other folders in the path required. """

    if os.path.isdir(folder):
        pass
    elif os.path.isfile(folder):
        raise OSError("%s exists as a regular file." % folder)
    else:
        parent, directory = os.path.split(folder)
        if parent and not os.path.isdir(parent):
            makedir(parent)
        if directory:
            os.mkdir(folder)
