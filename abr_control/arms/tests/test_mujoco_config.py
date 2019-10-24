import numpy as np
import pytest

from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.interfaces.mujoco import Mujoco

from .dummy_mujoco_arm import TwoJoint


# TODO
# def test_connect


# TODO
# def test_load_state


def test_g():
    test_arm = TwoJoint()
    # set up the Mujoco interface and config
    robot_config = arm('twojoint')
    interface = Mujoco(robot_config=robot_config)
    interface.connect()

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.g(q), test_arm.g(q))


# TODO
# def test_dJ():


def test_J():
    test_arm = TwoJoint(L0=.2, L1=.4)
    # set up the Mujoco interface and config
    robot_config = arm('twojoint')
    interface = Mujoco(robot_config=robot_config)
    interface.connect()

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(
                robot_config.J('link0', q, object_type='geom'),
                test_arm.J_link0(q))
            assert np.allclose(
                robot_config.J('link1', q, object_type='geom'),
                test_arm.J_link1(q))
            assert np.allclose(
                robot_config.J('link2', q, object_type='geom'),
                test_arm.J_link2(q))
            assert np.allclose(
                robot_config.J('EE', q), test_arm.J_EE(q))


def test_M():
    test_arm = TwoJoint()
    robot_config = arm('twojoint')
    interface = Mujoco(robot_config=robot_config)
    interface.connect()

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.M(q), test_arm.M(q))


def test_R():
    test_arm = TwoJoint()
    robot_config = arm('twojoint')
    interface = Mujoco(robot_config=robot_config)
    interface.connect()

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(
                robot_config.R('link1', q), test_arm.R_link0(q))
            assert np.allclose(
                robot_config.R('joint0', q), test_arm.R_joint0(q))
            assert np.allclose(
                robot_config.R('link2', q), test_arm.R_link1(q))
            assert np.allclose(
                robot_config.R('joint1', q), test_arm.R_joint1(q))
            assert np.allclose(
                robot_config.R('EE', q), test_arm.R_link2(q))
            assert np.allclose(
                robot_config.R('EE', q), test_arm.R_EE(q))


# TODO
# def test_quaternion():


def test_C():
    test_arm = TwoJoint()
    robot_config = arm('twojoint')
    interface = Mujoco(robot_config=robot_config)
    interface.connect()

    with pytest.raises(NotImplementedError):
        q_vals = np.linspace(0, 2*np.pi, 15)
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
    # set up the Mujoco interface and config
    robot_config = arm('twojoint')
    interface = Mujoco(robot_config=robot_config)
    interface.connect()

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(
                robot_config.Tx('link0', q, object_type='geom'),
                test_arm.Tx_link0(q),
                atol=1e-5)
            assert np.allclose(
                robot_config.Tx('joint0', q, object_type='joint'),
                test_arm.Tx_joint0(q),
                atol=1e-5)
            assert np.allclose(
                robot_config.Tx('link1', q, object_type='geom'),
                test_arm.Tx_link1(q),
                atol=1e-5)
            assert np.allclose(
                robot_config.Tx('joint1', q, object_type='joint'),
                test_arm.Tx_joint1(q))
            assert np.allclose(
                robot_config.Tx('link2', q, object_type='geom'),
                test_arm.Tx_link2(q))
            assert np.allclose(
                robot_config.Tx('EE', q),
                test_arm.Tx_EE(q))


# TODO:
# def test_T_inv():
