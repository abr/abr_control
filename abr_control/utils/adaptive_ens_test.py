import numpy as np
import matplotlib.pyplot as plt

import abr_jaco2

import nengo
from nengo.utils.matplotlib import rasterplot

import nengolib

folder = 'data'
run_to_plot = 0

robot_config = abr_jaco2.robot_config_neural()

q = robot_config.scaledown('q', np.load('%s/q.npz' % folder)['q'])
dq = robot_config.scaledown('dq', np.load('%s/dq.npz' % folder)['dq'])
print('q shape: ', q.shape)

model = nengo.Network(seed=10)
with model:

    in_q = nengo.Node(
        lambda t: q[int(t*1000)])
    in_dq = nengo.Node(
        lambda t: dq[int(t*1000)])

    eval_points = (nengolib.stats.ScatteredHypersphere(surface=False) if
                   nengolib is not None else None)
    encoders = (nengolib.stats.ScatteredHypersphere(surface=True) if
                nengolib is not None else None)
    adapt_ens = nengo.Ensemble(
        seed=10,
        n_neurons=20000,
        dimensions=12,
        encoders=encoders,
        eval_points=eval_points,
        intercepts=nengo.dists.Uniform(.5, 1),)

    nengo.Connection(in_q, adapt_ens[:6],
        function=lambda x: x / np.linalg.norm(x))
    nengo.Connection(in_dq, adapt_ens[6:],
        function=lambda x: x / np.linalg.norm(x))

    probe = nengo.Probe(adapt_ens.neurons, synapse=None)

sim = nengo.Simulator(model)
sim.run(2)

fig = plt.figure(figsize=(6, 6))
plt.subplot(211)
for ii in range(6):
    plt.plot(q[1:, ii])

plt.subplot(212)
rasterplot(sim.trange(), sim.data[probe])#[:, :500])
plt.show()
