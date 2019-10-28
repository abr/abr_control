import numpy as np
import pytest

from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.interfaces.mujoco import Mujoco

from .dummy_mujoco_arm import TwoJoint


# TODO
# def test_connect


# TODO
# def test_load_state



def test_M(plt):
    test_arm = TwoJoint()
    robot_config = arm('onejoint')
    interface = Mujoco(robot_config=robot_config, visualize=False)
    interface.connect()

    muj_Mi = robot_config.sim.model.body_inertia
    muj_m = robot_config.sim.model.body_mass
    link1 = robot_config.sim.model.body_name2id('link1')

    test_Mi = np.asarray(test_arm.M_LINKS)[:, 3:, 3:]
    test_m = np.asarray(test_arm.M_LINKS)[:, :3, :3]

    # check inertias
    assert np.allclose(muj_Mi[link1], np.diag(test_Mi[1]), atol=1e-5)
    # check masses
    assert np.allclose(muj_m[link1], np.diag(test_m[1])[0], atol=1e-5)
    # check our diag masses match
    assert np.allclose(np.mean(np.diag(test_m[1])), np.diag(test_m[1])[0], atol=1e-5)

    # create the inertia matrices for each link using the above information for Mujoco
    muj_M1 = np.diag([muj_m[link1], muj_m[link1], muj_m[link1],
                      muj_Mi[link1][0], muj_Mi[link1][1], muj_Mi[link1][2]])

    # get the test inertia matrices, at this point they have passed the assertion
    test_M1 = test_arm.M_LINKS[1]

    # store data for visual comparison when debugging
    plot = True
    obj_type = 'geom'
    q1s = []
    mujs = []
    mines = []
    if plot:
        a = [0,0,0,0]
        for ii in range(4):
            a[ii] = plt.subplot(4,1,ii+1)

    q_vals = np.linspace(0, 2*np.pi, 50)
    for q1 in q_vals:
        print('---------------')
        q = q1

        # get jacobians
        muj_J1 = np.copy(robot_config.J('link1', q, object_type='geom'))
        test_J1 = np.copy(test_arm.J_link1(q)[:, 0][:, None])

        assert np.allclose(
            robot_config.J('link0', q, object_type='geom'),
            test_arm.J_link0(q)[:, 0][:, None])
        assert np.allclose(
            robot_config.J('link1', q, object_type='geom'),
            test_arm.J_link1(q)[:, 0][:, None])

        assert np.allclose(muj_J1, test_J1)

        muj_Mx = np.copy(robot_config.M(q))
        # test_Mx = test_arm.M(q)
        # test_Mx1 = (
        #     np.dot(
        #         np.dot(
        #             np.asarray(robot_config.J(
        #                 q=q, name='link1', object_type=obj_type)).T,
        #             np.asarray(test_M1)
        #         ),
        #         np.asarray(robot_config.J(q=q, name='link1', object_type=obj_type)
        test_Mx1 = (np.dot(np.dot(muj_J1.T, np.asarray(test_M1)), muj_J1))

        test_Mx = test_Mx1

        # # if not plot:
        assert np.allclose(muj_Mx, test_Mx)

        # else:
        # q1s.append(np.copy(q1))
        # mujs.append(muj_Mx)
        # mines.append(test_Mx)

    # if plot:
    #     mujs = np.asarray(mujs)
    #     q1s = np.asarray(q1s)
    #     mines = np.asarray(mines)
    #     print('muj shape: ', mujs[:,0,0].shape)
    #     print(q1s.shape)
    #     print('muj shape: ', mujs[0].shape)
    #     a[0].plot(q1s, mujs[:,0,0], 'g')
    #     a[1].plot(q1s, mujs[:,0,1], 'g')
    #     a[2].plot(q1s, mujs[:,1,0], 'g')
    #     a[3].plot(q1s, mujs[:,1,1], 'g')
    #     a[0].plot(q1s, mines[:,0,0], '--b')
    #     a[1].plot(q1s, mines[:,0,1], '--b')
    #     a[2].plot(q1s, mines[:,1,0], '--b')
    #     a[3].plot(q1s, mines[:,1,1], '--b')
    #     break
    #
    #
