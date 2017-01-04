"""
For benchmarking the generation time of the inertia matrix Mq
against the three different robot_config options. Basic, with
simplify, and with both simplify and cythonize.
"""
import matplotlib.pyplot as plt
import numpy as np
import os
import time

import abr_control

# TODO: add flags for cache_size=None and turning on/off gmpy2

os.environ["SYMPY_USE_CACHE"] = "yes"
os.environ["SYMPY_CACHE_SIZE"] = "None"

config_names = []
configs = []
# initialize our robot config
config_names.append('basic')
configs.append(abr_control.arms.jaco2.config(
    regenerate_functions=True))
# NOTE: with simplification this takes on the order of a day
# config_names.append('simplify')
# configs.append(abr_control.arms.jaco2.config(
#     regenerate_functions=True, use_simplify=True))
config_names.append('cython')
configs.append(abr_control.arms.jaco2.config(
    regenerate_functions=True, use_cython=True))
# NOTE: with simplification this takes on the order of a day
# config_names.append('simplify + cython')
# configs.append(abr_control.arms.jaco2.config(
#     regenerate_functions=True, use_simplify=True, use_cython=True))

# create a target based on initial arm position
q = np.zeros(configs[0].num_joints)

gen_times = np.empty(len(configs))
print('Testing generation time...')
for jj, config in enumerate(configs):
    start_time = time.time()
    Mq = config.Mq(q=q)
    gen_times[jj] = time.time() - start_time
    print('%s Mq gen time: %.5f' %
            (config_names[jj], gen_times[jj]))
print("")

# make sure they all generate the same answers
for ii in range(100):
    q = np.random.random(configs[0].num_joints)
    Mq = []
    for config in configs:
        Mq.append(config.Mq(test_points[-1], q=q))
    for ii in range(len(configs) - 1):
        assert np.allclose(Mq[ii].flatten(),
                           Mq[ii+1].flatten(),
                           atol=1e-7)

run_times = np.empty(len(configs))
print('Testing execution time...')
for jj, config in enumerate(configs):
    num = 10000
    times = np.empty(num)
    for kk in range(num):
        start_time = time.time()
        Mq = config.Mq(q=q)
        times[kk] = (time.time() - start_time)
    run_times[jj] = np.sum(times) / num
    print('Average Mq time: %.5f' % run_times[jj])
print("")

plt.figure(figsize=(5,5))
plt.subplot(2, 1, 1)
plt.bar(range(len(configs)), gen_times, .9)
plt.title('Function generation time')
plt.xticks(range(len(configs)), config_names)
plt.ylabel('Time (s)')

plt.subplot(2, 1, 2)
plt.bar(range(len(configs)), run_times, .9)
plt.title('Average execution time of %i trials' % num)
plt.xticks(range(len(configs)), config_names)
plt.ylabel('Time (s)')

plt.tight_layout()

plt.savefig('results/inertiamatrix.png')
plt.savefig('results/inertiamatrix.pdf')
plt.show()
