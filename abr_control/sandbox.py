import numpy as np

import abr_control

# initialize our robot config for the ur5
regenerate = True
robot_config = abr_control.arms.ur5.config_kinadapt3.robot_config(
    regenerate_functions=regenerate)
q = np.zeros(6)
name = 'link2'
print('T: ', robot_config._calc_T(name, lambdify=False, regenerate=regenerate))
print('J: ', robot_config._calc_J(name, lambdify=False, regenerate=regenerate))
print('dJ: ', robot_config._calc_dJ(name, lambdify=False, regenerate=regenerate))
