import pytest
import timeit

import matplotlib.pyplot as plt
import numpy as np

from abr_control.arms import twojoint, threejoint, ur5, jaco2
from abr_control.controllers import OSC
from abr_control.utils import transformations


@pytest.mark.parametrize(
    "arm, ctrlr_dof",
    (
        (ur5, [True, True, True, True, True, True]),
        (jaco2, [True, True, True, True, True, False]),
    ),
)
def test_velocity_limiting(arm, ctrlr_dof):
    # Derivation worked through at studywolf.wordpress.com/2016/11/07/ +
    # velocity-limiting-in-operational-space-control/
    robot_config = arm.Config()

    kp = 10
    ko = 8
    kv = 4
    vmax = 1
    ctrlr = OSC(
        robot_config, kp=kp, ko=ko, kv=kv, ctrlr_dof=ctrlr_dof, vmax=[vmax, vmax]
    )

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


@pytest.mark.parametrize(
    "arm, ctrlr_dof",
    (
        (ur5, [True, True, True, True, True, True]),
        (jaco2, [True, True, True, True, True, False]),
    ),
)
def test_Mx(arm, ctrlr_dof):
    robot_config = arm.Config(use_cython=True)
    ctrlr = OSC(robot_config, ctrlr_dof=ctrlr_dof)

    for ii in range(100):
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


def calc_distance(Qe, Qd):
    dr = Qe[0] * Qd[1:] - Qd[0] * Qe[1:] - np.cross(Qd[1:], Qe[1:])
    return np.linalg.norm(dr, 2)


@pytest.mark.parametrize(
    "arm, orientation_algorithm",
    ((threejoint, 0), (threejoint, 1), (ur5, 0), (ur5, 1), (jaco2, 0), (jaco2, 1),),
)
def test_calc_orientation_forces(arm, orientation_algorithm):
    robot_config = arm.Config(use_cython=False)
    ctrlr = OSC(robot_config, orientation_algorithm=orientation_algorithm)

    for ii in range(100):
        q = np.random.random(robot_config.N_JOINTS) * 2 * np.pi
        quat = robot_config.quaternion("EE", q=q)

        theta = np.pi / 2
        axis = np.array([0, 0, 1])
        quat_rot = transformations.unit_vector(
            np.hstack([np.cos(theta / 2), np.sin(theta / 2) * axis])
        )
        quat_target = transformations.quaternion_multiply(quat, quat_rot)
        target_abg = transformations.euler_from_quaternion(quat_target, axes="rxyz")

        # calculate current position quaternion
        R = robot_config.R("EE", q=q)
        quat_1 = transformations.unit_vector(transformations.quaternion_from_matrix(R))
        dist1 = calc_distance(quat_1, np.copy(quat_target))

        # calculate current position quaternion with u_task added
        u_task = ctrlr._calc_orientation_forces(target_abg, q=q)

        dq = np.dot(
            np.linalg.pinv(robot_config.J("EE", q)), np.hstack([np.zeros(3), u_task])
        )
        q_2 = q - dq * 0.001  # where 0.001 represents the time step
        R_2 = robot_config.R("EE", q=q_2)
        quat_2 = transformations.unit_vector(
            transformations.quaternion_from_matrix(R_2)
        )

        dist2 = calc_distance(quat_2, np.copy(quat_target))

        assert np.abs(dist2) < np.abs(dist1)
