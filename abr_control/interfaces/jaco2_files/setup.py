from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
  
setup(
  cmdclass = {'build_ext': build_ext},
  ext_modules=[
    Extension("jaco2_rs485", 
              sources=["jaco2_cython.pyx", "jaco2_rs485.cpp"],
              language="c++",
              extra_compile_args=["-Wl,--no-as-needed"],
              extra_link_args=["-ldl"],
              )]
)
