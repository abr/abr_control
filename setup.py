import io
import runpy
import os

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext as _build_ext


class build_ext(_build_ext):
    def finalize_options(self):
        _build_ext.finalize_options(self)
        # Prevent numpy from thinking it is still in its setup process:
        __builtins__.__NUMPY_SETUP__ = False
        import numpy
        self.include_dirs.append(numpy.get_include())


def read(*filenames, **kwargs):
    encoding = kwargs.get('encoding', 'utf-8')
    sep = kwargs.get('sep', '\n')
    buf = []
    for filename in filenames:
        with io.open(filename, encoding=encoding) as f:
            buf.append(f.read())
    return sep.join(buf)


root = os.path.dirname(os.path.realpath(__file__))
version = runpy.run_path(
    os.path.join(root, 'abr_control', 'version.py'))['version']
setup_requires = ["setuptools>=18.0", "cython", "numpy"]

setup(
    name="abr_control",
    version=version,
    author="Applied Brain Research",
    author_email="travis.dewolf@appliedbrainresearch.com",
    packages=find_packages(),
    include_package_data=True,
    scripts=[],
    url="https://github.com/abr/abr_control",
    license="Free for non-commercial use",
    description="A library for controlling and interfacing with robots.",
    long_description=read('README.rst'),
    install_requires=setup_requires + [
        "cloudpickle", "sympy", "nengo", "matplotlib", "scipy",
    ],
    setup_requires=setup_requires,
    cmdclass={'build_ext': build_ext},
    ext_modules=[
        Extension(
            "abr_control.arms.threelink.arm_files.py3LinkArm",
            sources=["abr_control/arms/threelink/arm_files/py3LinkArm.pyx"],
            language="c++",
        )
    ],
)
