import numpy as np
import pytest

pytest.importorskip("mujoco")

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
    robot_config = arm("twojoint")
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
    robot_config = arm("twojoint")
    interface = Mujoco(robot_config=robot_config, visualize=False)
    interface.connect()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(
                robot_config.J("link0", q, object_type="geom"), test_arm.J_link0(q)
            )
            assert np.allclose(
                robot_config.J("link1", q, object_type="geom"), test_arm.J_link1(q)
            )
            assert np.allclose(
                robot_config.J("link2", q, object_type="geom"), test_arm.J_link2(q)
            )
            assert np.allclose(robot_config.J("EE", q), test_arm.J_EE(q))


# def test_M(plt):
#     test_arm = TwoJoint(L0=0.2, L1=0.4)
#     robot_config = arm("twojoint")
#     interface = Mujoco(robot_config=robot_config, visualize=False)
#     interface.connect()
#
#     muj_I = robot_config.sim.model.body_inertia
#     muj_m = robot_config.sim.model.body_mass
#     link1 = robot_config.sim.model.body_name2id("link1")
#     link2 = robot_config.sim.model.body_name2id("link2")
#
#     test_I = np.vstack([np.diag(I) for I in np.asarray(test_arm.M_LINKS)[:, 3:, 3:]])
#     test_m = np.asarray(test_arm.M_LINKS)[:, 0, 0]
#
#     # check inertias
#     assert np.allclose(muj_I[link1], test_I[1], atol=1e-5)
#     assert np.allclose(muj_I[link2], test_I[2], atol=1e-5)
#     # check masses
#     assert np.allclose(muj_m[link1], test_m[1], atol=1e-5)
#     assert np.allclose(muj_m[link2], test_m[2], atol=1e-5)
#
#     q_vals = np.linspace(0, 2 * np.pi, 50)
#     for q0 in q_vals:
#         for q1 in q_vals:
#             print("---------------")
#             q0 = q_vals[12]
#             q = [q0, q1]
#
#             # get jacobians
#             muj_J1 = robot_config.J("link1", q)
#             muj_J2 = robot_config.J("link2", q)
#
#             test_J1 = test_arm.J_link1(q)
#             test_J2 = test_arm.J_link2(q)
#
#             assert np.allclose(muj_J1, test_J1)
#             assert np.allclose(muj_J2, test_J2)
#
#             muj_M = robot_config.M(q)
#
#             # following the formulas from Todorov's Featherstone slide 4
#             print(muj_I)
#             Ic1 = np.diag(muj_I[link1])
#             Ic2 = np.diag(muj_I[link2])
#
#             print("Ic1: \n", Ic1)
#             print("Ic2: \n", Ic2)
#
#             # c1 = robot_config.Tx('link1', q=q)
#             # c2 = robot_config.Tx('link2', q=q)
#             c1 = robot_config.sim.model.body_ipos[link1]
#             c2 = robot_config.sim.model.body_ipos[link2]
#
#             print("simple1: ", robot_config.sim.model.body_simple[link1])
#             print("simple2: ", robot_config.sim.model.body_simple[link2])
#             print("sameframe1: ", robot_config.sim.model.body_sameframe[link1])
#             print("sameframe2: ", robot_config.sim.model.body_sameframe[link2])
#             print("pos1: ", robot_config.sim.model.body_pos[link1])
#             print("pos2: ", robot_config.sim.model.body_pos[link2])
#             print("quat1: ", robot_config.sim.model.body_quat[link1])
#             print("quat2: ", robot_config.sim.model.body_quat[link2])
#             print("ipos1: ", robot_config.sim.model.body_ipos[link1])
#             print("ipos2: ", robot_config.sim.model.body_ipos[link2])
#             print("iquat1: ", robot_config.sim.model.body_iquat[link1])
#             print("iquat2: ", robot_config.sim.model.body_iquat[link2])
#
#             # function to generate a matrix such that
#             # np.dot(tilde(x), y) = np.cross(x, y)
#             tilde = lambda x: np.array(
#                 [[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]]
#             )
#             Io1 = Ic1 - muj_m[link1] * np.dot(tilde(c1), tilde(c1))
#             Io2 = Ic2 - muj_m[link2] * np.dot(tilde(c2), tilde(c2))
#
#             def gen_Io_tilde(Io, m, c_tilde):
#                 Io_tilde = np.zeros((6, 6))
#                 Io_tilde[:3, :3] = Io
#                 Io_tilde[:3, 3:] = m * c_tilde
#                 Io_tilde[3:, :3] = m * c_tilde.T
#                 Io_tilde[3:, 3:] = m * np.eye(3)
#                 return Io_tilde
#
#             Io_tilde1 = gen_Io_tilde(Io1, muj_m[link1], tilde(c1))
#             Io_tilde2 = gen_Io_tilde(Io2, muj_m[link2], tilde(c2))
#
#             print("Io_tilde1: ", Io_tilde1)
#             print("J1: ", muj_J1)
#             print("J2: ", muj_J2)
#
#             test_Mx1 = np.dot(muj_J1.T, np.dot(Io_tilde1, muj_J1))
#             test_Mx2 = np.dot(muj_J2.T, np.dot(Io_tilde2, muj_J2))
#
#             print("test_Mx1: ", test_Mx1)
#             print("test_Mx2: ", test_Mx2)
#
#             test_Mx = test_Mx1 + test_Mx2
#
#             print("muj: ", [float("%0.5f" % val) for val in muj_M.flatten()])
#             print("test: ", [float("%0.5f" % val) for val in test_Mx.flatten()])
#
#             assert np.allclose(muj_M, test_Mx)


def test_R():
    test_arm = TwoJoint()
    robot_config = arm("twojoint")
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


def test_C():
    test_arm = TwoJoint()
    robot_config = arm("twojoint")
    interface = Mujoco(robot_config=robot_config, visualize=False)
    interface.connect()

    with pytest.raises(NotImplementedError):
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
    # set up the Mujoco interface and config
    robot_config = arm("twojoint")
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
