""" moves jaco using position control then
moves to target using force control"""
import numpy as np
import time
import abr_control
import abr_jaco2

# ---------- INITIALIZATION ----------
# initialize our robot config for the ur5
robot_config = abr_jaco2.robot_config(
    regenerate_functions=False, use_cython=True,
    use_simplify=False, hand_attached=False)

# instantiate the REACH controller
ctrlr = abr_control.controllers.osc(
    robot_config, kp=1, kv=.1, vmax=0.5)

# run controller once to generate functions / take care of overhead
# outside of the main loop (so the torque mode isn't exited)
ctrlr.control(np.zeros(6), np.zeros(6),target_x=np.zeros(3))

# create our VREP interface
interface = abr_jaco2.interface(robot_config)
interface.connect()

# home position tilted forward to avoid hitting camera mount used before home
# position so moving to home will just be tilting the arm back
prehome_pos = np.array([4.36332, 2.61799, 3.14159, 4.71239, 0.0, 0.0], dtype="float32")
home_pos = np.array([4.36332, 3.14159, 3.14159, 4.71239, 0.0, 0.0], dtype="float32")
pos_in_deg = np.array([250.0, 140.0, 100.0, 230.0, 40.0, 0.0], dtype="float32")
# ---------- MAIN BODY ----------
# Move to home position
#interface.apply_q(prehome_pos)
interface.apply_q(pos_in_deg)
target_xyz = np.array([0.169, 0.330, 0.85])
t_feedback = interface.get_torque_load()
torque_load = np.array(t_feedback['torque_load'], dtype="float32")
print('load torques: ', torque_load)
try:
    count = 0
    loop_count = 0
    print('target_xyz: ', target_xyz)
    start_loop = True

    while 1:
        start = time.time()
        loop_count += 1
        # get arm feedback
        feedback = interface.get_feedback()
        q = (np.array(feedback['q'])%360) * np.pi / 180.0
        dq = np.array(feedback['dq']) * np.pi / 180.0
        # generate a control signal
        u = ctrlr.control(q=q, dq=dq,
                          target_x=target_xyz)

        # calculate tooltip endpoint in cartesian coordinates
        ee_xyz_control = robot_config.Tx('EE', q=q)

        if start_loop is True:
            interface.init_force_mode(expected_torque = torque_load)
            start_loop = False

        interface.apply_u(np.array(u, dtype='float32'))

        euclidean_err = np.sqrt(np.sum((target_xyz - ee_xyz_control)**2))

        # break once end-effector is within 5mm of the target

        if loop_count % 1000 == 0:
            print('euclidean error: ', euclidean_err)
            print('loop speed: ', time.time() - start)

        #if euclidean_err < .01:
        #    count += 1
        #    if count > 200:
        #        print("At Target Position")
        #        break

finally:
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position)
    interface.disconnect()
