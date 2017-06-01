from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy

setup(
  cmdclass = {'build_ext': build_ext},
  ext_modules=[Extension("abr_control/arms/threelink/arm_files/py3LinkArm",
               sources=["abr_control/arms/threelink/arm_files/py3LinkArm.pyx"],
               language="c++",
               include_dirs=[numpy.get_include()]),],
)
