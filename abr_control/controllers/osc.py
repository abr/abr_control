import numpy as np


class controller:
    """ Implements an operational space controller (OSC)
    """

    def __init__(self, robot_config, kp=1, kv=None, vmax=0.5,
                 null_control=True):

        self.robot_config = robot_config

        # proportional gain term
        self.kp = kp
        # derivative gain term
        self.kv = np.sqrt(self.kp) if kv is None else kv
        # velocity limit of the end-effector
        self.vmax = vmax
        self.lamb = self.kp / self.kv

        self.null_control = null_control

    def control(self, q, dq,
                target_pos, target_vel=np.zeros(3),
                target_quat=None, target_w=np.zeros(3),
                mask=[1, 1, 1, 0, 0, 0],
                ref_frame='EE', offset=[0, 0, 0]):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_pos np.array: the target end-effector position values, post-mask
        target_vel np.array: the target end-effector velocity, post-mask
        target_quat np.array: the target orientation as a quaternion
                              in the form [w, x, y, z]
        target_w np.array: the target angular velocities
        mask list: indicates the [x, y, z, roll, pitch, yaw] values
                   to be controlled, 1 = control, 0 = ignore
        ref_frame string: the frame of reference of control point
        offset list: point of interest from the frame of reference
        """
        # calculate the end-effector position information
        xyz = self.robot_config.Tx(ref_frame, q, x=offset)

        # calculate the Jacobian for the end effector
        J = self.robot_config.J(ref_frame, q, x=offset)
        J = J[:3]

        # calculate the end-effector linear and angular velocity
        full_velocity_signal = np.dot(J, dq)
        dx = full_velocity_signal[:3]  # linear velocity
        # w = full_velocity_signal[3:]  # angular velocity

        # calculate the inertia matrix in joint space
        M = self.robot_config.M(q)

        # apply mask to Jacobian before
        # J *= np.array(mask).reshape(6, 1)
        # calculate the inertia matrix in task space
        M_inv = np.linalg.inv(M)
        Mx_inv = np.dot(J, np.dot(M_inv, J.T))
        # using the rcond to set singular values < thresh to 0
        # is slightly faster than doing it manually with svd
        Mx = np.linalg.pinv(Mx_inv, rcond=.01)

        u_task = np.zeros(3)  # task space control signal before masking

        # generate the position control signal in task space if a target
        # position was provided, and the mask includes positions
        if target_pos is not None and np.sum(mask[:3]) > 0:
            # calculate the position error
            x_tilde = np.array(xyz - target_pos)
            # x_tilde = np.array([0, 0, 0.05])

            if self.vmax is not None:
                # implement velocity limiting
                sat = self.vmax / (self.lamb * np.abs(x_tilde))
                if np.any(sat < 1):
                    index = np.argmin(sat)
                    unclipped = self.kp * x_tilde[index]
                    clipped = self.kv * self.vmax * np.sign(x_tilde[index])
                    scale = np.ones(3, dtype='float32') * clipped / unclipped
                    scale[index] = 1
                else:
                    scale = np.ones(3, dtype='float32')

                u_task[:3] = -self.kv * (dx - target_vel -
                                         np.clip(sat / scale, 0, 1) *
                                         -self.lamb * scale * x_tilde)
            else:
                # generate (x,y,z) force without velocity limiting)
                # u_task[:3] = -self.kp * x_tilde + self.kv * (target_vel - dx)
                u_task[:3] = -self.kp * x_tilde

        # # generate the orientation control signal in task space if a target
        # # orientation was provided, and the mask includes orientation angles
        # if target_quat is not None and np.sum(mask[3:]) > 0:
        #     # get the quaternion describing orientation of ref_frame
        #     quat = self.robot_config.orientation(ref_frame, q)
        #
        #     # calculate the error between the two quaternions as
        #     # described in (Nakanishi et al, 2008)
        #     target_e = np.array([
        #         [0, -target_quat[3], target_quat[2]],
        #         [target_quat[3], 0, -target_quat[1]],
        #         [-target_quat[2], target_quat[1], 0]])
        #     error_quat = (target_quat[0] * quat[1:] -
        #                   quat[0] * target_quat[1:] +
        #                   np.dot(target_e, quat[1:]))
        #
        #     ko = self.kp
        #     ka = np.sqrt(ko)
        #     u_task[3:] = (ka * (target_w - w) - ko * error_quat)
        # # apply mask
        # u_task *= mask

        # incorporate task space inertia matrix
        # self.training_signal = np.dot(J.T, np.dot(Mx, u_task))

        dJ = self.robot_config.dJ(ref_frame, q=q, dq=dq)
        dJ = dJ[:3]
        self.training_signal = np.dot(J.T, np.dot(Mx, (u_task - np.dot(dJ, dq))))
        # self.training_signal = np.dot(J.T, np.dot(Mx, u_task))

        # TODO: This is really awkward, but how else to get out
        # this signal for dynamics adaptation training?
        if self.vmax is None:
            self.training_signal -= np.dot(M, dq)

        # cancel out effects of gravity
        u = self.training_signal - self.robot_config.g(q=q)
        # cancel out centripetal and Coriolis effects
        u -= self.robot_config.C(q=q, dq=dq)

        if self.null_control is True:
            # calculate the null space filter
            nkp = self.kp #* .1
            nkv = np.sqrt(nkp)

            q_des = np.zeros(self.robot_config.num_joints, dtype='float32')
            dq_des = np.zeros(self.robot_config.num_joints, dtype='float32')

            # calculated desired joint angle acceleration using rest angles
            for ii in range(1, self.robot_config.num_joints):
                if self.robot_config.rest_angles[ii] is not None:
                    q_des[ii] = (
                        ((self.robot_config.rest_angles[ii] - q[ii]) + np.pi) %
                        (np.pi*2) - np.pi)
                    dq_des[ii] = dq[ii]
            u_null = np.dot(M, (nkp * q_des - nkv * dq_des))

            Jbar = np.dot(M_inv, np.dot(J.T, Mx))
            null_filter = (np.eye(self.robot_config.num_joints) -
                           np.dot(J.T, Jbar.T))

            u += np.dot(null_filter, u_null)

        return u
