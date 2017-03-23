"""
A basic script for connecting and moving the arm to 4 targets.
The end-effector and target positions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position.
"""
import numpy as np
import signal
import sys
import traceback

import abr_control

# initialize our robot config
robot_config = abr_control.arms.jaco2.config(
    use_cython=True, hand_attached=True)
# instantiate the controller
ctrlr = abr_control.controllers.osc(
    robot_config, kp=20, kv=4, vmax=1, null_control=True)

# create our vrep interface
interface = abr_control.interfaces.vrep(
    robot_config=robot_config, dt=.001)
interface.connect()

# set up arrays for tracking end-effector and target position
ee_track = []
target_track = []

target_index = 0
at_target_count = 0

# list of targets to move to
targets = [[-.4, .22, .7],
           [-.4, -.22, .7],
           [.4, -.22, .7],
           [.4, .22, .7],
           [-.4, .22, .7]]
target_xyz = targets[0]


def on_exit(signal, frame):
    """ A function for plotting the end-effector trajectory and error """
    global ee_track, target_track
    ee_track = np.array(ee_track)
    target_track = np.array(target_track)

    import matplotlib.pyplot as plt

    # plot targets and trajectory of end-effectory in 3D
    abr_control.utils.plotting.plot_trajectory(
        ee_track, target_track, save_file_name=None)
    plt.show()
    sys.exit()

# call on_exit when ctrl-c is pressed
signal.signal(signal.SIGINT, on_exit)

offset = [0, 0, 0]  # m
print('Moving to first target: ', target_xyz)
try:
    feedback = interface.get_feedback()
    interface.set_xyz(name='target', xyz=target_xyz)
    ctr = 0
    while 1:
        ctr += 1
        feedback = interface.get_feedback()

        u = ctrlr.control(q=feedback['q'],
                          dq=feedback['dq'],
                          offset=offset,
                          target_pos=target_xyz)
        print('u: ', [float('%.3f' % val) for val in u])
        interface.send_forces(np.array(u, dtype='float32'))

        tooltip_refframe = 'link4'
        interface.set_xyz('tooltip', robot_config.Tx(
            tooltip_refframe, x=offset, q=feedback['q']))
        quaternion = robot_config.orientation(
            tooltip_refframe, q=feedback['q'])
        angles = abr_control.utils.transformations.euler_from_quaternion(
            quaternion, axes='rxyz')
        interface.set_orientation('tooltip', angles)

        ee_xyz = robot_config.Tx('EE', q=feedback['q'], x=offset)
        error = np.sqrt(np.sum((ee_xyz - target_xyz)**2))
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

        ee_track.append(np.copy(ee_xyz))
        target_track.append(np.copy(target_xyz))

        if ctr % 1000 == 0:
            print('error: ', error)

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.disconnect()
