import numpy as np
import pytest

pytest.importorskip("mujoco_py")

from abr_control.arms.mujoco_config import MujocoConfig as arm  # pylint: disable=C0413
from abr_control.interfaces.mujoco import Mujoco  # pylint: disable=C0413

from .dummy_mujoco_arm import TwoJoint  # pylint: disable=C0413


# TODO
# def test_connect


# TODO
# def test_load_state


def test_g():
    test_arm = TwoJoint()
    # set up the Mujoco interface and config
    robot_config = arm("twojoint", use_sim_state=False)
    interface = Mujoco(robot_config=robot_config, visualize=False)
    interface.connect()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.g(q), test_arm.g(q))


def test_J():
    test_arm = TwoJoint(L0=0.2, L1=0.4)
    # set up the Mujoco interface and config
    robot_config = arm("twojoint", use_sim_state=False)
    interface = Mujoco(robot_config=robot_config, visualize=False)
    interface.connect()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(
                robot_config.J("link0", q, object_type="geom"), test_arm.J_link0(q)
            )
            print('config J: \n', robot_config.J("link1", q, object_type="geom"))
            print('J: \n', test_arm.J_link1(q))
            return
            assert np.allclose(
                robot_config.J("link1", q, object_type="geom"), test_arm.J_link1(q)
            )
            assert np.allclose(
                robot_config.J("link2", q, object_type="geom"), test_arm.J_link2(q)
            )
            assert np.allclose(robot_config.J("EE", q), test_arm.J_EE(q))


def test_M(plt):
    test_arm = TwoJoint()
    robot_config = arm("twojoint", use_sim_state=False)
    interface = Mujoco(robot_config=robot_config , visualize=False)
    interface.connect()

    muj_I = robot_config.sim.model.body_inertia
    muj_m = robot_config.sim.model.body_mass
    link1 = robot_config.sim.model.body_name2id("link1")
    link2 = robot_config.sim.model.body_name2id("link2")

    test_I = np.vstack([np.diag(I) for I in np.asarray(test_arm.M_LINKS)[:, 3:, 3:]])
    test_m = np.asarray(test_arm.M_LINKS)[:, 0, 0]

    # check inertias match
    assert np.allclose(muj_I[link1], test_I[1], atol=1e-5)
    assert np.allclose(muj_I[link2], test_I[2], atol=1e-5)
    # check masses match
    assert np.allclose(muj_m[link1], test_m[1], atol=1e-5)
    assert np.allclose(muj_m[link2], test_m[2], atol=1e-5)

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            muj_M = robot_config.M(q)
            H = test_arm.M(q)
            assert np.allclose(muj_M, H)


def test_R():
    test_arm = TwoJoint()
    robot_config = arm("twojoint", use_sim_state=False)
    interface = Mujoco(robot_config=robot_config, visualize=False)
    interface.connect()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(
                robot_config.R("link1", q), test_arm.R_link1(q), atol=1e-5
            )
            assert np.allclose(
                robot_config.R("link2", q), test_arm.R_link2(q), atol=1e-5
            )
            assert np.allclose(robot_config.R("EE", q), test_arm.R_EE(q), atol=1e-5)


# TODO
# def test_quaternion():

# TODO
# def test_C():

# TODO
# def test_T():


def test_Tx():
    test_arm = TwoJoint()
    # set up the Mujoco interface and config
    robot_config = arm("twojoint", use_sim_state=False)
    interface = Mujoco(robot_config=robot_config, visualize=False)
    interface.connect()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(
                robot_config.Tx("link0", q, object_type="geom"),
                test_arm.Tx_link0(q),
                atol=1e-5,
            )
            assert np.allclose(
                robot_config.Tx("joint0", q, object_type="joint"),
                test_arm.Tx_joint0(q),
                atol=1e-5,
            )
            assert np.allclose(
                robot_config.Tx("link1", q, object_type="geom"),
                test_arm.Tx_link1(q),
                atol=1e-5,
            )
            assert np.allclose(
                robot_config.Tx("joint1", q, object_type="joint"), test_arm.Tx_joint1(q)
            )
            assert np.allclose(
                robot_config.Tx("link2", q, object_type="geom"), test_arm.Tx_link2(q)
            )
            assert np.allclose(robot_config.Tx("EE", q), test_arm.Tx_EE(q))
