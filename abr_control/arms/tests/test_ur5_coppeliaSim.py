from abr_control.arms import ur5
from abr_control.controllers import OSC
from abr_control.interfaces import COPPELIASIM

robot_config = ur5.Config()
ctrlr = OSC(robot_config, kp=20,
            # control (x, y, z) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof=[True, True, True, False, False, False])
interface = COPPELIASIM(robot_config)

interface.connect()

target_xyz = [.2, .2, .5]  # in metres
target_orientation = [0, 0, 0]  # Euler angles, relevant when controlled
for ii in range(1000):
    feedback = interface.get_feedback()  # returns a dictionary with q, dq
    u = ctrlr.generate(
        q=feedback['q'],
        dq=feedback['dq'],
        target=np.hstack([target_xyz, target_orientation]))
    interface.send_forces(u)  # send forces and step CoppeliaSim sim forward

interface.disconnect()