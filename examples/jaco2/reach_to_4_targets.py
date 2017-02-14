"""
A basic script for connecting and moving the arm to 4 targets.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position.
"""
import numpy as np

import abr_control

# initialize our robot config
robot_config = abr_control.arms.jaco2.config(
    regenerate_functions=True, use_cython=True,
    use_simplify=False)
# instantiate the controller
ctrlr = abr_control.controllers.osc(
    robot_config, kp=10, kv=3, vmax=.1, null_control=True)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
ctrlr.control(np.zeros(6), np.zeros(6), target_pos=np.zeros(3))

# create our vrep interface
interface = abr_control.interfaces.vrep(
    robot_config=robot_config, dt=.001)
interface.connect()

# set up arrays for tracking end-effector and target position
ee_track = []
targets_track = []

count = 0
target_index = 0
at_target_count = 0

# list of targets to move to
targets = [[-.4, .22, .7],
           [-.4, -.22, .7],
           [.4, -.22, .7],
           [.4, .22, .7],
           [-.4, .22, .7]]
target_xyz = targets[0]
print('Moving to first target: ', target_xyz)

try:
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', q=feedback['q'])
    interface.set_xyz(name='target', xyz=target_xyz)
    ctr = 0
    while 1:
        ctr += 1
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', q=feedback['q'])

        u = ctrlr.control(q=feedback['q'], dq=feedback['dq'],
                          target_pos=target_xyz)
        print('u: ', [float('%.3f' % val) for val in u])
        interface.apply_u(np.array(u, dtype='float32'))

        error = np.sqrt(np.sum((hand_xyz - target_xyz)**2))
        if error < .01:
            # if we're at the target, start count
            # down to moving to the next target
            at_target_count += 1
            if at_target_count >= 200:
                target_index += 1
                if target_index > len(targets):
                    break
                else:
                    target_xyz = targets[target_index]
                    interface.set_xyz(name='target', xyz=target_xyz)
                    print('Moving to next target: ', target_xyz)
                at_target_count = 0

        # set orientation of hand object to match EE
        quaternion = robot_config.orientation('EE', q=feedback['q'])
        angles = abr_control.utils.transformations.euler_from_quaternion(
            quaternion, axes='rxyz')
        interface.set_orientation('hand', angles)

        ee_track.append(hand_xyz)
        targets_track.append(target_xyz)
        count += 1

        if ctr % 1000 == 0:
            print('error: ', error)

except Exception as e:
    print(e)

finally:
    # close the connection to the arm
    interface.disconnect()

    if count > 0:  # i.e. if it successfully ran
        ee_track = np.array(ee_track)
        targets_track = np.array(targets_track)
        # plot targets and trajectory of end-effectory in 3D
        abr_control.utils.plotting.plot_trajectory(
            ee_track, targets_track, save_file_name='trajectory')
