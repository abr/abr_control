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
    robot_config = arm("twojoint")
    interface = Mujoco(robot_config=robot_config)
    interface.connect()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            assert np.allclose(robot_config.g(q), test_arm.g(q))


# TODO
# def test_dJ():


def test_J():
    test_arm = TwoJoint(L0=0.2, L1=0.4)
    # set up the Mujoco interface and config
    robot_config = arm("twojoint")
    interface = Mujoco(robot_config=robot_config)
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


def test_M(plt):
    test_arm = TwoJoint()
    robot_config = arm("twojoint")
    interface = Mujoco(robot_config=robot_config)  # , visualize=False)
    interface.connect()

    muj_Mi = robot_config.sim.model.body_inertia
    muj_m = robot_config.sim.model.body_mass
    link1 = robot_config.sim.model.body_name2id("link1")
    link2 = robot_config.sim.model.body_name2id("link2")

    test_Mi = np.asarray(test_arm.M_LINKS)[:, 3:, 3:]
    test_m = np.asarray(test_arm.M_LINKS)[:, :3, :3]

    # check inertias
    assert np.allclose(muj_Mi[link1], np.diag(test_Mi[1]), atol=1e-5)
    assert np.allclose(muj_Mi[link2], np.diag(test_Mi[2]), atol=1e-5)
    # check masses
    assert np.allclose(muj_m[link1], np.diag(test_m[1])[0], atol=1e-5)
    assert np.allclose(muj_m[link2], np.diag(test_m[2])[0], atol=1e-5)
    # check our diag masses match
    assert np.allclose(np.mean(np.diag(test_m[1])), np.diag(test_m[1])[0], atol=1e-5)
    assert np.allclose(np.mean(np.diag(test_m[2])), np.diag(test_m[1])[2], atol=1e-5)

    # create the inertia matrices for each link using the above information for Mujoco
    muj_M1 = np.diag(
        [
            muj_m[link1],
            muj_m[link1],
            muj_m[link1],
            muj_Mi[link1][0],
            muj_Mi[link1][1],
            muj_Mi[link1][2],
        ]
    )
    muj_M2 = np.diag(
        [
            muj_m[link2],
            muj_m[link2],
            muj_m[link2],
            muj_Mi[link2][0],
            muj_Mi[link2][1],
            muj_Mi[link2][2],
        ]
    )

    # print('muj M1: \n', muj_M1)
    # print('muj M2: \n', muj_M2)

    # get the test inertia matrices, at this point they have passed the assertion
    test_M1 = test_arm.M_LINKS[1]
    test_M2 = test_arm.M_LINKS[2]

    # print('test M1: \n', test_M1)
    # print('test M2: \n', test_M2)

    # store data for visual comparison when debugging
    plot = True
    obj_type = "geom"
    q1s = []
    mujs = []
    mines = []
    if plot:
        a = [0, 0, 0, 0]
        for ii in range(4):
            a[ii] = plt.subplot(4, 1, ii + 1)

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            print("---------------")
            q0 = q_vals[12]
            q = [q0, q1]

            # get jacobians
            muj_J1 = robot_config.J("link1", q)
            muj_J2 = robot_config.J("link2", q)

            # print('muj-COMJ1\n', robot_config.sim.data.get_body_jacbodycom)

            test_J1 = test_arm.J_link1(q)
            test_J2 = test_arm.J_link2(q)

            # print('sim ipos: ', robot_config.sim.model.body_ipos)
            # print('sim iquat: ', robot_config.sim.model.body_iquat)
            # print('body simple: ', robot_config.sim.model.body_simple)
            # print('body sameframe: ', robot_config.sim.model.body_sameframe)

            print("muj-J2\n", muj_J2)
            print("test-J2\n", test_J2)

            assert np.allclose(muj_J1, test_J1)
            assert np.allclose(muj_J2, test_J2)

            muj_M = robot_config.M(q)

            test_Mx1 = np.dot(muj_J1.T, np.dot(muj_M1, muj_J1))
            test_Mx2 = np.dot(muj_J2.T, np.dot(muj_M2, muj_J2))

            test_Mx = test_Mx1 + test_Mx2

            print("muj: ", [float("%0.5f" % val) for val in muj_M.flatten()])
            print("test: ", [float("%0.5f" % val) for val in test_Mx.flatten()])

            # if not plot:
            assert np.allclose(muj_M, test_Mx)

            # else:
            #     q1s.append(np.copy(q1))
            #     mujs.append(muj_M)
            #     mines.append(test_M)

        # if plot:
        #     mujs = np.asarray(mujs)
        #     q1s = np.asarray(q1s)
        #     mines = np.asarray(mines)
        #     a[0].plot(q1s, mujs[:,0,0], 'g')
        #     a[1].plot(q1s, mujs[:,0,1], 'g')
        #     a[2].plot(q1s, mujs[:,1,0], 'g')
        #     a[3].plot(q1s, mujs[:,1,1], 'g')
        #     a[0].plot(q1s, mines[:,0,0], '--b')
        #     a[1].plot(q1s, mines[:,0,1], '--b')
        #     a[2].plot(q1s, mines[:,1,0], '--b')
        #     a[3].plot(q1s, mines[:,1,1], '--b')
        #     break
        break


def test_R():
    test_arm = TwoJoint()
    robot_config = arm("twojoint")
    interface = Mujoco(robot_config=robot_config)
    interface.connect()

    q_vals = np.linspace(0, 2 * np.pi, 50)
    for q0 in q_vals:
        for q1 in q_vals:
            q = [q0, q1]
            print("\n----Q0----: ", q0)
            print("----Q1----: ", q0)
            # print('MUJOCO: \n', robot_config.R('link0', q))
            # print('MOCK: \n', test_arm.R_link0(q))
            # assert np.allclose(
            #     robot_config.R('link0', q), test_arm.R_link0(q))
            # assert np.allclose(
            #     robot_config.R('joint0', q), test_arm.R_joint0(q))
            print("LINK1")
            print("MUJOCO: \n", robot_config.R("link1", q))
            print("MOCK: \n", test_arm.R_link1(q))
            assert np.allclose(
                robot_config.R("link1", q), test_arm.R_link1(q), atol=1e-5
            )
            # assert np.allclose(
            #     robot_config.R('joint1', q), test_arm.R_joint1(q))
            print("LINK2")
            print("MUJOCO: \n", robot_config.R("link2", q))
            print("MOCK: \n", test_arm.R_link2(q))
            assert np.allclose(
                robot_config.R("link2", q), test_arm.R_link2(q), atol=1e-5
            )
            print("EE")
            print("MUJOCO: \n", robot_config.R("EE", q))
            print("MOCK: \n", test_arm.R_EE(q))
            assert np.allclose(robot_config.R("EE", q), test_arm.R_EE(q), atol=1e-5)


# TODO
# def test_quaternion():


def test_C():
    test_arm = TwoJoint()
    robot_config = arm("twojoint")
    interface = Mujoco(robot_config=robot_config)
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
    interface = Mujoco(robot_config=robot_config)
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


# TODO:
# def test_T_inv():
