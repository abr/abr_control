import io
import runpy
import os

from setuptools import find_packages, setup


def read(*filenames, **kwargs):
    encoding = kwargs.get('encoding', 'utf-8')
    sep = kwargs.get('sep', '\n')
    buf = []
    for filename in filenames:
        with io.open(filename, encoding=encoding) as f:
            buf.append(f.read())
    return sep.join(buf)


root = os.path.dirname(os.path.realpath(__file__))
version = runpy.run_path(os.path.join(root, 'abr_control',
                                      'version.py'))['version']
description = "ABR Control"
long_description = read('README.rst')

url = "https://github.com/abr/control"
setup(
    name="abr_control",
    version=version,
    author="Applied Brain Research",
    author_email="travis.dewolf@appliedbrainresearch.com",
    packages=find_packages(),
    include_package_data=True,
    scripts=[],
    url=url,
    license="",
    description=description,
    long_description=long_description,
    install_requires=["numpy", "sympy", "cloudpickle"],
)
