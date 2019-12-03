import os
import zipfile
import requests


def check_and_download(name, google_id, force_download=False):
    """
    Checks if the meshes folder exists in the xml directory
    If not it will ask the user if they want to download them
    to be able to proceed

    Parameters
    ----------
    name: string
        the file or directory to download
    google_id: string
        the google id that points to the location of the zip file.
        This should be stored in the xml or config file
    force_download: boolean, Optional (Default: False)
        True to skip checking if the file or folder exists
    """
    files_missing = False

    if force_download:
        files_missing = True
    else:
        # check if the provided name is a file or folder
        print('checking for : ', name)
        if not os.path.isfile(name) and not os.path.isdir(name):
            files_missing = True

    if files_missing:
        yes = ['y', 'yes']
        no = ['n', 'no']
        answered = False
        question = 'Download mesh and texture files to run sim? (y/n): '
        while not answered:
            reply = str(input(
                'Download mesh and texture files to run sim?' +' (y/n): ')).lower().strip()

            if reply[0] in yes:
                print('Downloading files...')
                name = name.split('/')
                name = '/'.join(s for s in name[:-1])
                download_files(google_id, name + '/tmp')
                print('Sim files saved to %s' % name)
                answered = True
            elif reply[0] in no:
                raise Exception('Please download the required files to run the demo')
            else:
                question = "Please Enter (y/n) "


def download_files(google_id, destination):
    print('destination: ', destination)

    def _get_confirm_token(response):
        for key, value in response.cookies.items():
            if key.startswith('download_warning'):
                return value

        return None

    def _save_response_content(response, destination):
        CHUNK_SIZE = 32768

        with open(destination, "wb") as f:
            for chunk in response.iter_content(CHUNK_SIZE):
                if chunk: # filter out keep-alive new chunks
                    f.write(chunk)

    def _extract_zip_files(zip_file):
        zip_file = '%s' % zip_file
        zipball = zipfile.ZipFile(zip_file)
        zipball.extractall(zip_file.split('tmp')[0])
        zipball.close()
        os.remove(zip_file)

    URL = "https://docs.google.com/uc?export=download"

    session = requests.Session()

    response = session.get(URL, params={'id': google_id}, stream=True)
    token = _get_confirm_token(response)

    if token:
        params = {'id': google_id, 'confirm': token}
        response = session.get(URL, params=params, stream=True)

    _save_response_content(response, destination)

    _extract_zip_files(destination)
