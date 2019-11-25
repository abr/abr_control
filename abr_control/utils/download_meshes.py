import os
import tarfile
import requests

def check_and_download(xml_dir, google_id, force_download=False):
    """
    Checks if the meshes folder exists in the xml directory
    If not it will ask the user if they want to download them
    to be able to proceed

    Parameters
    ----------
    xml_dir: string
        the directory that has the xml configuration file
    google_id: string
        the google id that points to the location of the
        meshes tarball. This should be in the xml file
        under a custom text tag with the name 'google_id'
    force_download: boolean, Optional (Default: False)
        True to skip checking if the meshes folder exists
    """
    files_missing = False

    if force_download:
        files_missing = True
    else:
        # print('looking for meshes folder in in %s' % xml_dir)
        if not os.path.isdir('%s/meshes'%xml_dir):
            files_missing = True

    if files_missing:
        yes = ['y', 'Y', 'yes']
        no = ['n', 'N', 'no']
        answered = False
        question = 'Download mesh and texture files to run sim? (y/n): '
        while not answered:
            reply = str(input(
                'Download mesh and texture files to run sim?' +' (y/n): ')).lower().strip()

            if reply[0] in yes:
                print('Downloading files...')
                download_files(google_id, '%s/meshes.tar' % xml_dir)
                print('Sim files saved to %s/meshes'%xml_dir)
                answered = True
            elif reply[0] in no:
                raise Exception('Please download the required files to run the demo')
            else:
                question = "Please Enter (y/n) "


def download_files(google_id, destination):
    def _get_confirm_token(response):
        for key, value in response.cookies.items():
            if key.startswith('download_warning'):
                return value

        return None

    def _save_response_content(response, destination):
        CHUNK_SIZE = 32768

        with open('%s' % destination, "wb") as f:
            for chunk in response.iter_content(CHUNK_SIZE):
                if chunk: # filter out keep-alive new chunks
                    f.write(chunk)

    def _extract_tar_files(tar_file):
        tar_file = '%s' % tar_file
        tarball = tarfile.open(tar_file)
        tarball.extractall(tar_file.split('meshes.tar')[0])
        tarball.close()
        os.remove(tar_file)

    URL = "https://docs.google.com/uc?export=download"

    session = requests.Session()

    response = session.get(URL, params={'id': google_id}, stream=True)
    token = _get_confirm_token(response)

    if token:
        params = {'id': google_id, 'confirm': token}
        response = session.get(URL, params=params, stream=True)

    _save_response_content(response, destination)

    _extract_tar_files(destination)
