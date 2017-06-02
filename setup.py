# from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import io
import runpy
import numpy
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
long_description = read('README.rst')

setup(
    name="abr_control",
    version=version,
    author="Applied Brain Research",
    author_email="travis.dewolf@appliedbrainresearch.com",
    packages=find_packages(),
    include_package_data=True,
    scripts=[],
    url="https://github.com/abr/control",
    license="Free for non-commercial use",
    description="A library for controlling and interfacing with robots.",
    long_description=long_description,
    install_requires=["numpy", "cloudpickle", "cython", "sympy"],
    cmdclass={'build_ext': build_ext},
    ext_modules=[
        Extension("abr_control/arms/threelink/arm_files/py3LinkArm",
            sources=["abr_control/arms/threelink/arm_files/py3LinkArm.pyx"],
            language="c++",
            include_dirs=[numpy.get_include()])])
