import pytest
import timeit

import matplotlib.pyplot as plt
import numpy as np

from abr_control.arms import twojoint, ur5, jaco2
from abr_control.controllers import OSC


@pytest.mark.parametrize('arm, ctrlr_dof', (
    (ur5, [True, True, True, True, True, True]),
    (jaco2, [True, True, True, True, True, False]),
    ))
def test_velocity_limiting(arm, ctrlr_dof):
    # Derivation worked through at studywolf.wordpress.com/2016/11/07/ +
    # velocity-limiting-in-operational-space-control/
    robot_config = arm.Config()

    kp = 10
    ko = 8
    kv = 4
    vmax = 1
    ctrlr = OSC(robot_config, kp=kp, ko=ko, kv=kv,
                ctrlr_dof=ctrlr_dof, vmax=[vmax, vmax])

    answer = np.zeros(6)
    # xyz < vmax, abg < vmax
    u_task_unscaled = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
    answer[:3] = kp * u_task_unscaled[:3]
    answer[3:] = ko * u_task_unscaled[3:]
    output = ctrlr._velocity_limiting(u_task_unscaled)
    assert np.allclose(output, answer, atol=1e-5)

    # xyz > vmax, abg < vmax
    u_task_unscaled = np.array([100.0, 100.0, 100.0, 0.05, 0.05, 0.05])
    answer[:3] = kv * np.sqrt(vmax / 3.0)
    answer[3:] = ko * u_task_unscaled[3:]
    output = ctrlr._velocity_limiting(u_task_unscaled)
    assert np.allclose(output, answer, atol=1e-5)

    # xyz < vmax, abg > vmax
    u_task_unscaled = np.array([0.05, 0.05, 0.05, 100.0, 100.0, 100.0])
    answer[:3] = kp * u_task_unscaled[:3]
    answer[3:] = kv * np.sqrt(vmax / 3.0)
    output = ctrlr._velocity_limiting(u_task_unscaled)
    assert np.allclose(output, answer, atol=1e-5)

    # xyz > vmax, abg > vmax
    u_task_unscaled = np.array([100.0, 100.0, 100.0, 100.0, 100.0, 100.0])
    answer[:3] = kv * np.sqrt(vmax / 3.0)
    answer[3:] = kv * np.sqrt(vmax / 3.0)
    output = ctrlr._velocity_limiting(u_task_unscaled)
    assert np.allclose(output, answer, atol=1e-5)


@pytest.mark.parametrize('arm, ctrlr_dof', (
    (ur5, [True, True, True, True, True, True]),
    (jaco2, [True, True, True, True, True, False]),
    ))
def test_Mx(arm, ctrlr_dof):
    robot_config = arm.Config(use_cython=True)
    ctrlr = OSC(robot_config, ctrlr_dof=ctrlr_dof)

    for ii in range(10):
        q = np.random.random(robot_config.N_JOINTS) * 2 * np.pi

        # test Mx is non-singular case
        J = np.eye(robot_config.N_JOINTS)
        M = robot_config.M(q=q)
        Mx, M_inv = ctrlr._Mx(M=M, J=J, threshold=1e-5)
        assert np.allclose(M, Mx, atol=1e-5)

        # test Mx in the singular case
        J = np.ones((6, robot_config.N_JOINTS))
        Mx, M_inv = ctrlr._Mx(M=M, J=J)
        U2, S2, Vh2 = np.linalg.svd(Mx)
        assert np.all(np.abs(S2[1:]) < 1e-10)
