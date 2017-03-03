import numpy as np
import time

import nengo


class KeepLearningSolver(nengo.solvers.Lstsq):
    """ Loads in weights from a file if they exist,
    otherwise returns weights generated with the Lstq solver."""

    def __init__(self, filename, weights=False, zero_default=True,
                 output_shape=None):
        super(KeepLearningSolver, self).__init__(weights=weights)
        self.filename = filename
        self.zero_default = zero_default
        self.output_shape = output_shape

    def __call__(self, A, Y, rng=None, E=None):
        import os

        tstart = time.time()
        info = {'rmses': None,
                'time': time.time() - tstart}

        if os.path.isfile('./%s' % self.filename):
            print('Loading weights from %s' % self.filename)
            weights = np.load(self.filename)['weights'][-1].T
            print(self.filename)

            if (weights.shape[0] != A.shape[1] or
                    weights.shape[1] != Y.shape[1]):
                raise Exception('Stored weights are not ' +
                                'correct shape for connection.')
        else:
            print('No weights file found, ')
            if self.zero_default is True:
                print('using zero matrix')
                print('A.shape: ', A.shape[1])
                print('Y.shape: ', Y.shape[0])
                weights = np.zeros((A.shape[1], self.output_shape))
            else:
                print('generating with Lstsq solver')
                weights, info = \
                    super(KeepLearningSolver, self).__call__(A, Y)

        return weights, info
