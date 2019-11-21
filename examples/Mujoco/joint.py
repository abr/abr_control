"""
A basic script for connecting and moving the arm to a target
configuration in joint space, offset from its starting position.
The simulation simulates 1500 time steps and then plots the results.
"""
import numpy as np
import traceback
import glfw

from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.interfaces.mujoco import Mujoco
from abr_control.controllers import Joint


# initialize our robot config
robot_config = arm('jaco2')

# create interface and connect
interface = Mujoco(robot_config=robot_config, dt=.001)
interface.connect()
interface.send_target_angles(robot_config.START_ANGLES)

# instantiate the REACH controller for the jaco2 robot
ctrlr = Joint(robot_config, kp=20, kv=10)

# make the target an offset of the current configuration
feedback = interface.get_feedback()
target = feedback['q'] + np.random.random(robot_config.N_JOINTS) * 2 - 1

# set up arrays for tracking end-effector and target position
q_track = []


try:
    count = 0
    print('\nSimulation starting...\n')
    while count < 15000:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()

        # calculate the control signal
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=target,
            )

        # add gripper forces
        u = np.hstack((u, np.ones(3)*0.05))

        # send forces into Mujoco, step the sim forward
        interface.send_forces(u)

        # track joint angles
        q_track.append(np.copy(feedback['q']))
        count += 1

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.disconnect()

    print('Simulation terminated...')

    q_track = np.array(q_track)
    if q_track.shape[0] > 0:
        import matplotlib.pyplot as plt
        plt.plot((q_track + np.pi) % (np.pi * 2) - np.pi)
        plt.gca().set_prop_cycle(None)
        plt.plot(np.ones(q_track.shape) *
                ((target + np.pi) % (np.pi * 2) - np.pi), '--')
        plt.legend(range(robot_config.N_JOINTS))
        plt.tight_layout()
        plt.show()
