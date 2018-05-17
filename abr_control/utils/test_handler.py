import numpy as np
import traceback
import abr_jaco2
from abr_control.controllers import OSC, path_planners
import nengo
from abr_control.utils import DataHandler, Target

robot_config = abr_jaco2.Config(use_cython=True, hand_attached=True)
zeros = np.zeros(robot_config.N_JOINTS)

ctrlr = OSC(robot_config, kp=25, kv=6, ki=0.2, vmax=1, null_control=False)
ctrlr.generate(zeros, zeros, np.zeros(3))

interface = abr_jaco2.Interface(robot_config)

robot_config.Tx('EE', q=zeros, x=robot_config.OFFSET)

target_xyz = []
# target_xyz = np.array([[.56, -.09, .95],
#                        # [.12, .15, .80],
#                        # [.80, .26, .61],
#                        [.38, .46, .81]])
for ii in range(0,10):
    target_xyz.append(np.copy(Target.random_target(
        r=[0.7, 0.9], theta=[3.14, 6.28], phi=[2.07, 4.21])))
print(target_xyz)
# instantiate path planner and set parameters
path = path_planners.SecondOrder(
    robot_config, n_timesteps=2000,
    w=1e4, zeta=2, threshold=0.08)
dt = 0.003

# connect to the jaco
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

q_t = []
dq_t = []
u_t = []
adapt_t = []
time_t = []
target_t = []
filtered_target_t = []
error_t = []
training_signal_t = []
input_signal_t = []
ee_xyz_t = []
int_err_t = []
friction_t = []

try:
    count = 0
    count_at_target = 0 #  must stay at target for 200 loop cycles for success
    target_index = 0

    feedback = interface.get_feedback()
    xyz = robot_config.Tx('EE', q=feedback['q'], x=robot_config.OFFSET)
    filtered_target = np.concatenate((xyz, np.array([0, 0, 0])), axis=0)

    interface.init_force_mode()
    while target_index < len(target_xyz):
        feedback = interface.get_feedback()
        xyz = robot_config.Tx('EE', q=feedback['q'], x=robot_config.OFFSET)

        filtered_target = path.step(
            state=filtered_target, target_pos=target_xyz[target_index])
        # generate the control signal
        u = ctrlr.generate(
            q=feedback['q'], dq=feedback['dq'],
            target_pos=filtered_target[:3],  # (x, y, z)
            target_vel=filtered_target[3:],  # (dx, dy, dz)
            offset=robot_config.OFFSET)

        # additional gain term due to high stiction of jaco base joint
        if u[0] > 0:
            u[0] *= 3.0
        else:
            u[0] *= 2.0

        interface.send_forces(np.array(u, dtype='float32'))
        error = np.sqrt(np.sum((xyz - target_xyz[target_index])**2))

        # print out the error every so often
        if count % 100 == 0:
            print('error: ', error)

        # track data
        q_t.append(np.copy(feedback['q']))
        dq_t.append(np.copy(feedback['dq']))
        u_t.append(np.copy(u))
        target_t.append(np.copy(target_xyz[target_index]))
        filtered_target_t.append(np.copy(filtered_target))
        error_t.append(np.copy(error))
        ee_xyz_t.append(np.copy(xyz))

        # if within 5cm of target for 200 time steps move to next target
        if error < .05:
            count_at_target += 1
            if count_at_target >= 200:
                count_at_target = 0
                target_index += 1

        count+=1

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()
    tracked_data = {'q': q_t, 'dq': dq_t, 'u': u_t, 'target': target_t,
            'filtered_target': filtered_target_t, 'error': error_t, 'ee_xyz':
            ee_xyz_t}

osc_params = ctrlr.params
rc_params = robot_config.params

dat = DataHandler(use_cache=True)

dat.save_data(tracked_data=tracked_data,
              session=None,
              run=None,
              test_name='arm_in_loop',
              test_group='testing_handler',
              overwrite=True,
              create=True)

dat.save(data=osc_params,
         save_location='testing_handler/arm_in_loop/OSC',
         overwrite=True,
         create=True)

dat.save(data=rc_params,
         save_location='testing_handler/arm_in_loop/robot_config',
         overwrite=True,
         create=True)

saved_data = dat.load_data(params=['q','doesnt_exist'],
             session=None,
             run=None,
             test_name='arm_in_loop',
             test_group='testing_handler')

print('SAVED DATA\n', saved_data)

keys = dat.get_keys('testing_handler/arm_in_loop/OSC')

saved_params = dat.load(params=keys,
        save_location='testing_handler/arm_in_loop/OSC')
for key in saved_params:
    print('%s: %s' %(key, saved_params[key]))

import matplotlib
matplotlib.use("TKAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

targets = np.array(target_xyz)
x = targets[:,0]
y = targets[:,1]
z = targets[:,2]
ax.scatter(x,y,z,c='r',marker='o')
plt.show()
