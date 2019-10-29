import shutil
import os

import numpy as np

from abr_control.arms import twojoint as arm

from .dummy_base_arm import TwoJoint


def test_generate_and_save_function():
    robot_config = arm.Config(use_cython=True)

    # remove any saved functions from cache folder
    if os.path.isfile(robot_config.config_folder + "/EE_T/setup.py"):
        shutil.rmtree(robot_config.config_folder + "/EE_T")
    # generate a function
    robot_config.T("EE", q=np.zeros(robot_config.N_JOINTS))
    # confirm file has been generated
    assert os.path.isfile(robot_config.config_folder + "/EE_T/setup.py")


def test_load_from_file():
    robot_config = arm.Config(use_cython=True)

    # generate a function
    robot_config.T("EE", q=np.zeros(robot_config.N_JOINTS))

    # if returning expression, function should be None
    expression, function = robot_config._load_from_file("EE_T", lambdify=False)
    assert expression is not None
    assert function is None

    # if returning function, expression should be None
    expression, function = robot_config._load_from_file("EE_T", lambdify=True)
    assert expression is None
    assert function is not None


def test_g():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.g(q), test_arm.g(q))


def test_dJ():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2 * np.pi, 15)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            for dq0 in q_vals:
                for dq1 in q_vals:
                    dq = [dq0, dq1]
                    assert np.allclose(
                        robot_config.dJ("link0", q, dq), test_arm.dJ_link0(q, dq)
                    )
                    assert np.allclose(
                        robot_config.dJ("joint0", q, dq), test_arm.dJ_joint0(q, dq)
                    )
                    assert np.allclose(
                        robot_config.dJ("link1", q, dq), test_arm.dJ_link1(q, dq)
                    )
                    assert np.allclose(
                        robot_config.dJ("joint1", q, dq), test_arm.dJ_joint1(q, dq)
                    )
                    assert np.allclose(
                        robot_config.dJ("link2", q, dq), test_arm.dJ_link2(q, dq)
                    )
                    assert np.allclose(
                        robot_config.dJ("EE", q, dq), test_arm.dJ_EE(q, dq)
                    )


def test_J():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.J("link0", q), test_arm.J_link0(q))
            assert np.allclose(robot_config.J("joint0", q), test_arm.J_joint0(q))
            assert np.allclose(robot_config.J("link1", q), test_arm.J_link1(q))
            assert np.allclose(robot_config.J("joint1", q), test_arm.J_joint1(q))
            assert np.allclose(robot_config.J("link2", q), test_arm.J_link2(q))
            assert np.allclose(robot_config.J("EE", q), test_arm.J_EE(q))


def test_M():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.M(q), test_arm.M(q))


def test_R():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.R("link0", q), test_arm.R_link0(q))
            assert np.allclose(robot_config.R("joint0", q), test_arm.R_joint0(q))
            assert np.allclose(robot_config.R("link1", q), test_arm.R_link1(q))
            assert np.allclose(robot_config.R("joint1", q), test_arm.R_joint1(q))
            assert np.allclose(robot_config.R("link2", q), test_arm.R_link2(q))
            assert np.allclose(robot_config.R("EE", q), test_arm.R_EE(q))


# TODO
# def test_quaternion():


def test_C():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2 * np.pi, 15)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            for dq0 in q_vals:
                for dq1 in q_vals:
                    dq = [dq0, dq1]
                    assert np.allclose(robot_config.C(q, dq), test_arm.C(q, dq))


# TODO
# def test_T():


def test_Tx():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.Tx("link0", q), test_arm.Tx_link0(q))
            assert np.allclose(robot_config.Tx("joint0", q), test_arm.Tx_joint0(q))
            assert np.allclose(robot_config.Tx("link1", q), test_arm.Tx_link1(q))
            assert np.allclose(robot_config.Tx("joint1", q), test_arm.Tx_joint1(q))
            assert np.allclose(robot_config.Tx("link2", q), test_arm.Tx_link2(q))
            assert np.allclose(robot_config.Tx("EE", q), test_arm.Tx_EE(q))


def test_T_inv():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.T_inv("link0", q), test_arm.T_inv_link0(q))
            assert np.allclose(
                robot_config.T_inv("joint0", q), test_arm.T_inv_joint0(q)
            )
            assert np.allclose(robot_config.T_inv("link1", q), test_arm.T_inv_link1(q))
            assert np.allclose(
                robot_config.T_inv("joint1", q), test_arm.T_inv_joint1(q)
            )
            assert np.allclose(robot_config.T_inv("link2", q), test_arm.T_inv_link2(q))
            assert np.allclose(robot_config.T_inv("EE", q), test_arm.T_inv_EE(q))
