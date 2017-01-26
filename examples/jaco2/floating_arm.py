# arm should remain in position, only gravity compensation
import numpy as np
import abr_control
import abr_jaco2

# ---------- INITIALIZATION ----------
# initialize our robot config for the ur5
robot_config = abr_jaco2.robot_config(
    regenerate_functions=True, use_cython=True, hand_attached=False)

# instantiate the REACH controller
ctrlr = abr_control.controllers.floating(robot_config)

# run controller once to generate functions / take care of overhead
# outside of the main loop (so the torque mode isn't exited)
ctrlr.control(np.zeros(6), np.zeros(6))  

# create our VREP interface
interface = abr_jaco2.interface(robot_config)
interface.connect()

try:
    start_loop = True

    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        q = (np.array(feedback['q']) % 360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0
        
        # generate a control signal
        u = ctrlr.control(q=q, dq=dq)

        if start_loop is True:
            interface.init_force_mode()
            start_loop = False

        interface.apply_u(np.array(u, dtype='float32'))      

finally:
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    interface.disconnect()
