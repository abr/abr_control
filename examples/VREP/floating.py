"""
A basic script for connecting to the arm and putting it in floating
mode, which only compensates for gravity. The end-effector position
is recorded and plotted when the script is exited (with ctrl-c).
"""
import numpy as np
import traceback

# from abr_control.arms import ur5 as arm
# from abr_control.arms import jaco2 as arm
from abr_control.arms import onelink as arm
from abr_control.controllers import Floating
from abr_control.interfaces import VREP

# initialize our robot config
robot_config = arm.Config(use_cython=True)
# instantiate the controller
ctrlr = Floating(robot_config)

# create the VREP interface and connect up
interface = VREP(robot_config, dt=.001)
interface.connect()

# set up arrays for tracking end-effector and target position
ee_track = []


try:
    feedback = interface.get_feedback()
    start = robot_config.Tx('EE', q=feedback['q'])
    while 1:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()
        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],)
        # send forces into VREP
        interface.send_forces(u)

        # calculate the position of the hand
        hand_xyz = robot_config.Tx('EE', q=feedback['q'])
        # track end effector position
        ee_track.append(hand_xyz)

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.disconnect()

    ee_track = np.array(ee_track)

    if ee_track.shape[0] > 0:
        import matplotlib.pyplot as plt
        from abr_control.utils.plotting import plot_3D
        plot_3D(ee_track)
        plt.show()
