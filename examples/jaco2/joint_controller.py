"""
A basic script for connecting and moving the arm to 4 targets.
The end-effector and target postions are recorded and plotted
once the final target is reached, and the arm has moved back
to its default resting position.
"""
import numpy as np

import abr_control

# initialize our robot config for neural controllers
robot_config = abr_control.arms.jaco2.config_link3(
    regenerate_functions=True, use_cython=True,
    use_simplify=False, hand_attached=False)
# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.joint(robot_config, kp=2, kv=1.5)

ctrlr.control(np.zeros(robot_config.num_joints),
              np.zeros(robot_config.num_joints),
              np.zeros(robot_config.num_joints))

# create our interface for the jaco2
interface = abr_control.interfaces.vrep(
    robot_config=robot_config, dt=.001)

target_pos = np.array([2.0, 2.75, 3.45], dtype='float32')
target_vel = None

# connect to the jaco
interface.connect()

# set up arrays for tracking end-effector and target position
ee_track = []
ctr = 0

try:
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', q=feedback['q'])
    while 1:
        ctr += 1
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx('EE', q=feedback['q'])

        u = ctrlr.control(q=feedback['q'], dq=feedback['dq'],
                          target_pos=target_pos, target_vel=target_vel)
        interface.apply_u(np.array(u, dtype='float32'))

        print('q: ', feedback['q'])
        # set orientation of hand object to match EE
        quaternion = robot_config.orientation('EE', q=feedback['q'])
        angles = abr_control.utils.transformations.euler_from_quaternion(
            quaternion, axes='rxyz')
        interface.set_orientation('hand', angles)

        ee_track.append(hand_xyz)

except Exception as e:
    print(e)

finally:
    # close the connection to the arm
    interface.disconnect()

    if ctr > 0:  # i.e. if it successfully ran
        import matplotlib.pyplot as plt
        # import seaborn

        ee_track = np.array(ee_track)
        abr_control.utils.plotting.plot_trajectory(
            ee_track, np.zeros(ee_track.shape))

        plt.tight_layout()
        plt.show()
