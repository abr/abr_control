from .version import version as __version__

from . import controllers

import sys
if sys.version_info > (3, 6, 9):
    raise ImportError(
        """
You are using Python version %s and abr_control
currently supports python up to 3.6.9.

Please create a new environment with python =<3.6.9
"""
% (sys.version))
