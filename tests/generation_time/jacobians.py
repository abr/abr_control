"""
For benchmarking the generation time of the Jacobian J
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
config_names.append('simplify')
configs.append(abr_control.arms.jaco2.config(
    regenerate_functions=True, use_simplify=True))
config_names.append('cython')
configs.append(abr_control.arms.jaco2.config(
    regenerate_functions=True, use_cython=True))
config_names.append('simplify + cython')
configs.append(abr_control.arms.jaco2.config(
    regenerate_functions=True, use_simplify=True, use_cython=True))

# create a target based on initial arm position
q = np.zeros(configs[0].num_joints)
test_points = ['joint%i' % ii for ii in range(3)]#configs[0].num_joints)]

gen_times = np.empty((len(test_points), len(configs)))
print('Testing generation time...')
for ii, joint in enumerate(test_points):
    for jj, config in enumerate(configs):
        start_time = time.time()
        J = config.J(joint, q=q)
        gen_times[ii, jj] = time.time() - start_time
        print('%s J gen time: %.5f' %
              (config_names[jj], gen_times[ii, jj]))
    print("")

# make sure they all generate the same answers
for ii in range(100):
    q = np.random.random(configs[0].num_joints)
    J = []
    for config in configs:
        J.append(config.J(test_points[-1], q=q))
    for ii in range(len(configs) - 1):
        assert np.allclose(J[ii].flatten(),
                           J[ii+1].flatten(),
                           atol=1e-7)

run_times = np.empty((len(test_points), len(configs)))
print('Testing execution time...')
for ii, joint in enumerate(test_points):
    for jj, config in enumerate(configs):
        num = 10000
        q = np.random.random(configs[0].num_joints)
        times = np.empty(num)
        for kk in range(num):
            start_time = time.time()
            J = config.J(joint, q=q)
            times[kk] = (time.time() - start_time)
        run_times[ii, jj] = np.sum(times) / num
        print('Average J time: %.5f' % run_times[ii, jj])
    print("")

plt.figure(figsize=(5,5))
plt.subplot(2, 1, 1)
plt.plot(gen_times)
plt.legend(config_names)
plt.title('Function generation time')
plt.xticks(range(len(test_points)))
plt.ylabel('Time (s)')

plt.subplot(2, 1, 2)
plt.plot(run_times)
plt.legend(config_names)
plt.title('Average execution time of %i trials' % num)
plt.xticks(range(len(test_points)))
plt.ylabel('Time (s)')
plt.xlabel('Num joints')

plt.tight_layout()

plt.savefig('results/jacobians.png')
plt.savefig('results/jacobians.pdf')
plt.show()
