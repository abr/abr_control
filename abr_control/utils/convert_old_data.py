from abr_control.utils import ConvertData, email_results
import subprocess
import os

def scp_data(command):
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()

cd = ConvertData(use_cache=True, db_name='dewolf2018neuromorphic')

cache_dir = '/saved_weights'
# ----------friction results----------
old_locs = ['%s/dewolf2017/no_weight/pd/pd_5pt_baseline'%cache_dir,
            '%s/dewolf2017/no_weight/pd/wear_124'%cache_dir,
            '%s/loihi2018/no_weight/pid/wear_26'%cache_dir,
            '%s/dewolf2017/no_weight/nengo/1000_neurons_x_1/wear_120'%cache_dir,
            '%s/loihi2018/no_weight/nengo_ocl/1000_neurons_x_1/wear_25'%cache_dir,
            '%s/chip_testing/no_weight/chip/wear15'%cache_dir
            ]

pre = 'friction_tests'
new_locs = ['%s/pd_no_friction'%pre,
            '%s/pd'%pre,
            '%s/pid'%pre,
            '%s/nengo_cpu1k'%pre,
            '%s/nengo_gpu1k'%pre,
            '%s/nengo_loihi1k'%pre
            ]
notes = []
for loc in old_locs:
    notes.append({'old_save_loc':loc})

# # scp data from work computer
# for loc in old_locs:
#     old_loc = '/home/pawel/.cache/abr_control%s'%loc
#     new_loc = '/home/pawel/.cache/abr_control%s'%loc
#     save_loc = new_loc.split('/')
#     save_loc = save_loc[:-1]
#     save_loc = '/'.join(save_loc)
#     if not os.path.exists(save_loc):
#         os.makedirs(save_loc)
#     command = 'scp -r pawel:%s %s'%(old_loc, new_loc)
#     print('command: ', command)
#     print('copying %s to %s'%(old_loc, new_loc))
#     scp_data(command=command)

# send_email(subject='friction tests copied over')
print('converting friction results to db')
cd.convert_data(old_location=old_locs, new_location=new_locs, notes=notes)
# send_email(subject='friction tests converted')

# ----------gradual friction results----------
old_locs = ['%s/loihi2018/no_weight/pd/pd_5pt_baseline'%cache_dir,
            '%s/chip_testing/no_weight/pd/gradual_wear1'%cache_dir,
            '%s/loihi2018/no_weight/pid/gradual_wear14'%cache_dir,
            '%s/loihi2018/no_weight/nengo/1000_neurons_x_20/gradual_wear23'%cache_dir,
            '%s/loihi2018/no_weight/nengo_ocl/1000_neurons_x_20/gradual_wear21'%cache_dir,
            '%s/loihi2018/no_weight/chip/gradual_wear24'%cache_dir]

pre = 'gradual_friction_tests'
new_locs = ['%s/pd_no_friction'%pre,
            '%s/pd'%pre,
            '%s/pid'%pre,
            '%s/nengo_cpu20k'%pre,
            '%s/nengo_gpu20k'%pre,
            '%s/nengo_loihi20k'%pre,
            ]
notes = []
for loc in old_locs:
    notes.append({'old_save_loc':loc})

# # scp data from work computer
# for loc in old_locs:
#     old_loc = '/home/pawel/.cache/abr_control%s'%loc
#     new_loc = '/home/pawel/.cache/abr_control%s'%loc
#     save_loc = new_loc.split('/')
#     save_loc = save_loc[:-1]
#     save_loc = '/'.join(save_loc)
#     if not os.path.exists(save_loc):
#         os.makedirs(save_loc)
#     command = 'scp -r pawel:%s %s'%(old_loc, new_loc)
#     print('command: ', command)
#     print('copying %s to %s'%(old_loc, new_loc))
#     scp_data(command=command)

# send_email(subject='gradual friction tests copied over')
print('converting gradual friction results to db')
cd.convert_data(old_location=old_locs, new_location=new_locs, notes=notes)
# send_email(subject='gradual friction tests converted')


# ----------weighted results----------
old_locs = ['%s/loihi2018/weighted/pd/not_weighted_19'%cache_dir,
            '%s/loihi2018/weighted/pd/weighted_18'%cache_dir,
            '%s/loihi2018/weighted/pid/weighted_29'%cache_dir,
            '%s/loihi2018/weighted/nengo/1000_neurons_x_1/weighted_46'%cache_dir,
            '%s/loihi2018/weighted/nengo_ocl/1000_neurons_x_1/weighted_34'%cache_dir,
            '%s/chip_testing/weighted/chip/weighted14'%cache_dir]

pre = 'weighted_tests'
new_locs = ['%s/pd_no_weight'%pre,
            '%s/pd'%pre,
            '%s/pid'%pre,
            '%s/nengo_cpu1k'%pre,
            '%s/nengo_gpu1k'%pre,
            '%s/nengo_loihi1k'%pre,
            ]
notes = []
for loc in old_locs:
    notes.append({'old_save_loc':loc})
# # scp data from work computer
# for loc in old_locs:
#     old_loc = '/home/pawel/.cache/abr_control%s'%loc
#     new_loc = '/home/pawel/.cache/abr_control%s'%loc
#     save_loc = new_loc.split('/')
#     save_loc = save_loc[:-1]
#     save_loc = '/'.join(save_loc)
#     if not os.path.exists(save_loc):
#         os.makedirs(save_loc)
#     command = 'scp -r pawel:%s %s'%(old_loc, new_loc)
#     print('command: ', command)
#     print('copying %s to %s'%(old_loc, new_loc))
#     scp_data(command=command)

# send_email(subject='weighted tests copied over')
print('converting gradual friction results to db')
cd.convert_data(old_location=old_locs, new_location=new_locs, notes=notes)
# send_email(subject='weighted tests converted')


