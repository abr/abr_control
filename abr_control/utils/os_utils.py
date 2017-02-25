import os

def makedir(folder):
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
