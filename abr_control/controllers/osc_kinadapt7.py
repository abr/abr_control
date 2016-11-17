import numpy as np

from . import osc
from . import osc_dynadapt
from .keeplearningsolver import KeepLearningSolver


class controller(osc_dynadapt.controller):
    """ Extension of the OSC controller that incorporates dynamics
    adaptation using a Nengo model
    """

    def __init__(self, robot_config, vmax=0.5, **kwargs):

        super(controller, self).__init__(robot_config, **kwargs)
        self.vmax = vmax

    def control(self, q, dq, target_state, object_xyz):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_state np.array: the current target for the end-effector
        """

        self.q = q
        self.dq = dq
        print('dq: ', dq)
        print(np.any(np.abs(dq) > 10))

        # calculate the inertia matrix in joint space
        Mq = self.robot_config.Mq(q)

        # check to see what controller should be used
        # if the arm has entered high velocity, switch to a controller
        # whose sole job is to stop the arm from moving
        if np.any(np.abs(dq) > 10):
            u = -self.kv * np.dot(Mq, dq)
        else:
            # calculate the effect of gravity in joint space
            Mq_g = self.robot_config.Mq_g(q)

            # calculate the Jacobian for the end effector
            JEE = self.robot_config.J('objectEE', q)

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
            dx = np.dot(JEE, dq)
            # implement velocity limiting
            lamb = self.kp / self.kv
            x_tilde = object_xyz - target_state[:3]
            sat = self.vmax / (lamb * np.abs(x_tilde))
            scale = np.ones(3)
            if np.any(sat < 1):
                index = np.argmin(sat)
                unclipped = self.kp * x_tilde[index]
                clipped = self.kv * self.vmax * np.sign(x_tilde[index])
                scale = np.ones(3) * clipped / unclipped
                scale[index] = 1
            u_xyz = -self.kv * (dx - target_state[3:] -
                                np.clip(sat / scale, 0, 1) *
                                -lamb * scale * x_tilde)
            # u_xyz = -self.kv * (dx - target_state[3:] -
            #                     np.clip(self.vmax / (lamb * np.abs(x_tilde)),
            #                             0, 1) * -lamb * x_tilde)
            # u_xyz = -self.kp * x_tilde - self.kv * dx
            u_xyz = np.dot(Mx, u_xyz)

            # add in gravity compensation, not included in training signal
            u = np.dot(JEE.T, u_xyz)- Mq_g

            # run the simulation to generate the adaptive signal
            self.training_signal = np.dot(
                np.linalg.pinv(JEE),
                (dx - target_state[3:] + 5 * (target_state[:3] - object_xyz)))
            self.sim.run(time_in_seconds=.001, progress_bar=False)
            # add in adaptive component
            print('u_adapt: ', [float('%.3f' % val) for val in self.u_adapt])
            u += self.u_adapt

            # secondary controller ish
            # calculate the null space filter
            Jdyn_inv = np.dot(Mx, np.dot(JEE, np.linalg.inv(Mq)))
            null_filter = (np.eye(self.robot_config.num_joints) -
                        np.dot(JEE.T, Jdyn_inv))

            q_des = np.zeros(self.robot_config.num_joints)
            dq_des = np.zeros(self.robot_config.num_joints)

            # calculated desired joint angle acceleration using rest angles
            for ii in range(1, self.robot_config.num_joints):
                if self.robot_config.rest_angles[ii] is not None:
                    q_des[ii] = (
                        ((self.robot_config.rest_angles[ii] - q[ii]) + np.pi) %
                        (np.pi*2) - np.pi)
                    dq_des[ii] = dq[ii]
            # only compensate for velocity for joints with a control signal
            nkp = self.kp
            nkv = np.sqrt(nkp)
            u_null = np.dot(Mq, (nkp * q_des - nkv * dq_des))

            u += np.dot(null_filter, u_null)

        return u
