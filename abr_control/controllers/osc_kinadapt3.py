import numpy as np

from . import osc
from . import osc_dynadapt
from .keeplearningsolver import KeepLearningSolver


# class controller(osc_dynadapt.controller):
class controller(osc.controller):
    """ Extension of the OSC controller that incorporates dynamics
    adaptation using a Nengo model
    """

    def __init__(self, robot_config):

        super(controller, self).__init__(robot_config,)
                                         # pes_learning_rate=1e-4)

    def control(self, q, dq, target_xyz, object_xyz):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_xyz np.array: the current target for the end-effector
        """

        self.q = q
        self.dq = q

        # calculate the Jacobian for the end effector
        JEE = self.robot_config.J('objectEE', q)

        # calculate the inertia matrix in joint space
        Mq = self.robot_config.Mq(q)

        # calculate the effect of gravity in joint space
        Mq_g = self.robot_config.Mq_g(q)

        # convert the mass compensation into end effector space
        Mx_inv = np.dot(JEE, np.dot(np.linalg.inv(Mq), JEE.T))
        svd_u, svd_s, svd_v = np.linalg.svd(Mx_inv)
        # cut off any singular values that could cause control problems
        singularity_thresh = .00025
        for i in range(len(svd_s)):
            svd_s[i] = 0 if svd_s[i] < singularity_thresh else \
                1./float(svd_s[i])
        # numpy returns U,S,V.T, so have to transpose both here
        Mx = np.dot(svd_v.T, np.dot(np.diag(svd_s), svd_u.T))

        # calculate desired force in (x,y,z) space
        u_xyz = np.dot(Mx, target_xyz - object_xyz)
        # transform into joint space
        u = (self.kp * np.dot(JEE.T, u_xyz) - np.dot(Mq, self.kv * dq))

        # # run the simulation to generate the adaptive signal
        # self.training_signal = np.copy(u)
        # self.sim.run(time_in_seconds=.001, progress_bar=False)
        # # add in adaptive component
        # u += self.u_adapt

        # add in gravity compensation
        u -= Mq_g

        # secondary controller ish
        # calculate the null space filter
        Jdyn_inv = np.dot(Mx, np.dot(JEE, np.linalg.inv(Mq)))
        null_filter = (np.eye(self.robot_config.num_joints) -
                       np.dot(JEE.T, Jdyn_inv))

        # calculate q0 target angle relative to object to prevent
        # getting stuck trying to reach the object while moving sideways
        target_angle = np.arctan2(target_xyz[1], target_xyz[0])
        # q0_des = (((target_angle - q[0]) + np.pi) %
        #           (np.pi*2) - np.pi)
        q0_des = ((target_angle - q[0]) % np.pi)

        # calculated desired joint angle acceleration using rest angles
        q_des = (((self.robot_config.rest_angles - q) + np.pi) %
                 (np.pi*2) - np.pi)
        # set desired angle for q0 to be relative to target position
        q_des[0] = q0_des
        u_null = np.dot(Mq, (self.kp * q_des - self.kv * dq))
        # let it be anywhere within np.pi / 4 range of target angle
        if q_des[0] < np.pi / 8.0 and q_des[0] > -np.pi / 8.0:
            u_null[0] = 0.0

        u += np.dot(null_filter, u_null)

        return u
