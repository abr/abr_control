"""
send joint angles and target information to Vrep to replay the run
with data saved in 'file_location'
"""
import numpy as np
import time

from abr_control.arms import jaco2
from abr_control.interfaces import VREP
from abr_control.utils.paths import cache_dir

run_num = 0
session_num = 0
loc = cache_dir+'/saved_weights/loihi2018/no_weight'
# test = '/nengo/1000_neurons_x_20/gradual_wear23'
test = '/nengo/1000_neurons_x_20/gradual_wear_recording'

# test = '/nengo_ocl/1000_neurons_x_20/gradual_wear21'
# test = '/nengo_ocl/1000_neurons_x_20/gradual_wear_recording'

# test = '/chip/gradual_wear24'
# test = '/chip/gradual_wear_recording'
data_loc = '/session%i/run%i_data/'%(session_num, run_num)
full_path = loc+test+data_loc
print('Replaying test %s'%full_path)

q = np.squeeze(np.load(full_path+'q%i.npz'%run_num)['q'])
filtered_target = np.squeeze(np.load(full_path+'target%i.npz'%run_num)['target'])
print(filtered_target.shape)
# create our config, interface, and connect to VREP
rc = jaco2.Config(
    use_cython=True, hand_attached=True)
interface = VREP(rc)

try:
    interface.connect()
    import timeit
    times=[]
    for ii in range(0, len(q), 5):
        start = timeit.default_timer()
        interface.send_target_angles(tuple(q[ii]))
        interface.set_xyz('target', tuple(filtered_target[ii]))
        #time.sleep(t[ii])
        times.append(timeit.default_timer()-start)

finally:
    interface.disconnect()
    print('average comm time: ', np.mean(times))
