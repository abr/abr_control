import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
import seaborn
import numpy as np
import scipy.special

import nengo
import nengolib
from abr_control.utils import DataHandler
loc = '/1lb_random_target/nengo_cpu_6/session000/run001'
dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')
data = dat.load(params=['input_signal', 'time'], save_location=loc)
inputs = data['input_signal']
time = data['time']

class AreaIntercepts(nengo.dists.Distribution):
    """ Generate an optimally distributed set of intercepts in
    high-dimensional space.
    """
    dimensions = nengo.params.NumberParam('dimensions')
    base = nengo.dists.DistributionParam('base')

    def __init__(self, dimensions, base=nengo.dists.Uniform(-1, 1)):
        super(AreaIntercepts, self).__init__()
        self.dimensions = dimensions
        self.base = base

    def __repr__(self):
        return ("AreaIntercepts(dimensions=%r, base=%r)" %
                (self.dimensions, self.base))

    def transform(self, x):
        sign = 1
        if x > 0:
            x = -x
            sign = -1
        return sign * np.sqrt(1 - scipy.special.betaincinv(
            (self.dimensions + 1) / 2.0, 0.5, x + 1))

    def sample(self, n, d=None, rng=np.random):
        s = self.base.sample(n=n, d=d, rng=rng)
        for i in range(len(s)):
            s[i] = self.transform(s[i])
        return s

class Triangular(nengo.dists.Distribution):
    left = nengo.params.NumberParam('dimensions')
    right = nengo.params.NumberParam('dimensions')
    mode = nengo.params.NumberParam('dimensions')

    def __init__(self, left, mode, right):
        super(Triangular, self).__init__()
        self.left = left
        self.right = right
        self.mode = mode

    def __repr__(self):
        return ("Triangular(left=%r, mode=%r, right=%r)" %
                (self.left, self.mode, self.right))

    def sample(self, n, d=None, rng=np.random):
        if d is None:
            return rng.triangular(self.left, self.mode, self.right, size=n)
        else:
            return rng.triangular(self.left, self.mode, self.right, size=(n, d))



#inputs = np.load('input_signal0.npz')['input_signal'][0]

net = nengo.Network(seed=0)
with net:
    intercepts = AreaIntercepts(
        dimensions=4,
        #base=nengo.dists.Uniform(0.75, 1),
        #base=nengo.dists.Uniform(-.1, 1),
        #base=nengo.dists.Uniform(-.2, -0.1),
        #base=nengo.dists.Uniform(-.5, -0.3),
        base=Triangular(-.9, -.9, 0.0),
        )
    ens = nengo.Ensemble(
        n_neurons=1000, dimensions=4,
        radius=np.sqrt(4), intercepts=intercepts)
    ens.encoders=(nengolib.stats.ScatteredHypersphere(surface=True))
sim = nengo.Simulator(net)

_, activity = nengo.utils.ensemble.tuning_curves(ens, sim, inputs)

proportion_active = np.mean(activity>0, axis=0)
print(np.mean(activity))

seaborn.distplot(proportion_active)
plt.savefig('proportion_active.png')

plt.figure()
plt.plot(np.arange(activity.shape[0])*np.cumsum(time), np.mean(activity>0, axis=1))
plt.title('proportion of neurons active')
plt.savefig('proportion_active_time.png')
plt.show()
