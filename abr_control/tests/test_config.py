import numpy as np

from abr_control.arms import twolink as arm

from .testarm import TwoJoint

def test_R():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(
                robot_config.R('link0', q), test_arm.R_link0(q))
            assert np.allclose(
                robot_config.R('joint0', q), test_arm.R_joint0(q))
            assert np.allclose(
                robot_config.R('link1', q), test_arm.R_link1(q))
            assert np.allclose(
                robot_config.R('joint1', q), test_arm.R_joint1(q))
            assert np.allclose(
                robot_config.R('link2', q), test_arm.R_link2(q))
            assert np.allclose(
                robot_config.R('EE', q), test_arm.R_EE(q))


def test_Tx():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(
                robot_config.Tx('link0', q), test_arm.Tx_link0(q))
            assert np.allclose(
                robot_config.Tx('joint0', q), test_arm.Tx_joint0(q))
            assert np.allclose(
                robot_config.Tx('link1', q), test_arm.Tx_link1(q))
            assert np.allclose(
                robot_config.Tx('joint1', q), test_arm.Tx_joint1(q))
            assert np.allclose(
                robot_config.Tx('link2', q), test_arm.Tx_link2(q))
            assert np.allclose(
                robot_config.Tx('EE', q), test_arm.Tx_EE(q))


def test_T_inv():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(
                robot_config.T_inv('link0', q), test_arm.T_inv_link0(q))
            assert np.allclose(
                robot_config.T_inv('joint0', q), test_arm.T_inv_joint0(q))
            assert np.allclose(
                robot_config.T_inv('link1', q), test_arm.T_inv_link1(q))
            assert np.allclose(
                robot_config.T_inv('joint1', q), test_arm.T_inv_joint1(q))
            assert np.allclose(
                robot_config.T_inv('link2', q), test_arm.T_inv_link2(q))
            assert np.allclose(
                robot_config.T_inv('EE', q), test_arm.T_inv_EE(q))


def test_J():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(
                robot_config.J('link0', q), test_arm.J_link0(q))
            assert np.allclose(
                robot_config.J('joint0', q), test_arm.J_joint0(q))
            assert np.allclose(
                robot_config.J('link1', q), test_arm.J_link1(q))
            assert np.allclose(
                robot_config.J('joint1', q), test_arm.J_joint1(q))
            assert np.allclose(
                robot_config.J('link2', q), test_arm.J_link2(q))
            assert np.allclose(
                robot_config.J('EE', q), test_arm.J_EE(q))


def test_dJ():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2*np.pi, 15)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            for dq0 in q_vals:
                for dq1 in q_vals:
                    dq = [dq0, dq1]
                    assert np.allclose(
                        robot_config.dJ('link0', q, dq),
                        test_arm.dJ_link0(q, dq))
                    assert np.allclose(
                        robot_config.dJ('joint0', q, dq),
                        test_arm.dJ_joint0(q, dq))
                    assert np.allclose(
                        robot_config.dJ('link1', q, dq),
                        test_arm.dJ_link1(q, dq))
                    assert np.allclose(
                        robot_config.J('joint1', q),
                        test_arm.J_joint1(q))
                    assert np.allclose(
                        robot_config.J('link2', q),
                        test_arm.J_link2(q))
                    assert np.allclose(
                        robot_config.J('EE', q),
                        test_arm.J_EE(q))


def test_M():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.M(q), test_arm.M(q))


def test_g():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.g(q), test_arm.g(q))


def test_C():
    test_arm = TwoJoint()
    robot_config = arm.Config()

    count = 0
    q_vals = np.linspace(0, 2*np.pi, 15)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            for dq0 in q_vals:
                for dq1 in q_vals:
                    dq = [dq0, dq1]
                    assert np.allclose(robot_config.C(q, dq), test_arm.C(q, dq))
