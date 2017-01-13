import numpy as np
import time

import abr_control

import os; os.environ["SYMPY_CACHE_SIZE"] = "None"

robot_config = abr_control.arms.jaco2.config(
    regenerate_functions=True)

start_time = time.time()
print('Calculating inertia matrix')
robot_config.Mq(q=np.zeros(6))
print('done in: %.6f seconds' % (time.time() - start_time))

# # ctrlr = abr_control.controllers.osc(
# #     robot_config=robot_config)
#
# name = 'EE'
# try:
#     # test out our orientation calculations
#     while 1:
#         feedback = interface.get_feedback()
#         print('q: ', feedback['q'])
#         xyz = robot_config.Tx(name, q=feedback['q'])
#
#         quaternion = robot_config.orientation(name, q=feedback['q'])
#         # the s means it's a static frame
#         angles = abr_control.utils.transformations.euler_from_quaternion(
#             quaternion, axes='sxyz')
#         print('angles: ', np.array(angles) * 180.0 / np.pi)
#
#         interface.set_xyz('hand', xyz)
#         interface.set_orientation('hand', angles)
#
#         time.sleep(1)
#
# finally:
#     # stop and reset the VREP simulation
#     interface.disconnect()
