try:
    from setuptools import setup
    from setuptools import Extension
except ImportError:
    from distutils.core import setup
    from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy as np

setup(
    cmdclass = {'build_ext': build_ext},
    ext_modules = [Extension('wrapped_module_1', ['wrapped_module_1.pyx', 'wrapped_code_1.c'],
                             extra_compile_args=['-std=c99'])],
    include_dirs = [np.get_include()],
        )