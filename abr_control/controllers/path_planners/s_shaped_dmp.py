"""
Running operational space control with a PyGame display, and using the pydmps
library to specify a trajectory for the end-effector to follow, in
this case, a circle.

To install the pydmps library, clone https://github.com/studywolf/pydmps
and run 'python setup.py develop'
"""
import numpy as np

try:
    import pydmps
except ImportError:
    print('\npydmps library required, see ' +
          'https://github.com/studywolf/pydmps\n')

import scipy.interpolate
from .dmp_filter import dmpFilter


class SShapedDmp(dmpFilter):
    def __init__(self):
        super(SShapedDmp, self).__init__()
        def gauss(a, b, c):
            return a * np.exp(-(x-b)**2/(2*c)**2)

        x = np.linspace(0, np.pi*6, 100)
        g = gauss(0.5, np.pi, 2)
        h = gauss(1, 3*np.pi, 2)
        i = gauss(0.5, 5*np.pi, 2)

        y_des = np.vstack([
            np.copy(x),
            np.cumsum(g - h + i)*0.01,
            np.copy(x)])

        self.dmps.imitate_path(y_des)

    @property
    def params(self):
        params = {'source': 'dmpFilter'}
        return params
