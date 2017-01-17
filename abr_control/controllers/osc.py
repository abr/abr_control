import numpy as np

import abr_control


class controller:
    """ Implements an operational space controller (OSC)
    """

    def __init__(self, robot_config, kp=100, kv=None, vmax=0.5):

        self.robot_config = robot_config

        # proportional gain term
        self.kp = kp
        # derivative gain term
        self.kv = np.sqrt(self.kp) if kv is None else kv
        # velocity limit of the end-effector
        self.vmax = vmax
        self.lamb = self.kp / self.kv

    def control(self, q, dq,
                target_x, target_dx=np.zeros(3),
                target_w=None, target_dw=None, w_axes='sxyz',
                mask=[1, 1, 1, 0, 0, 0],
                ref_frame='EE', offset=[0, 0, 0]):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_x np.array: the target end-effector position values, post-mask
        target_dx np.array: the target end-effector velocity, post-mask
        target_w np.array: the target Euler angles
        target_dw np.array: the target angular velocities
        w_axes string: the order of rotation of the Euler angles with
                       rotation matrices. Default matches VREP.
                       First character is either s for static or
                       r for relative.
        mask list: indicates the [x, y, z, alpha, beta, gamma] values
                   to be controlled, 1 = control, 0 = ignore
        ref_frame string: the frame of reference of control point
        offset list: point of interest from the frame of reference
        """
        # calculate the end-effector position information
        xyz = self.robot_config.Tx(ref_frame, q, x=offset)

        # calculate the Jacobian for the end effector
        JEE = self.robot_config.J(ref_frame, q, x=offset)

        # apply mask to isolate DOFs of interest
        JEE = np.array([JEE[ii] for ii in range(6) if mask[ii] == 1])

        # calculate the end-effector velocity information
        dx = np.dot(JEE, dq)

        # calculate the inertia matrix in joint space
        Mq = self.robot_config.Mq(q)

        # calculate the effect of gravity in joint space
        Mq_g = self.robot_config.Mq_g(q)

        # calculate the inertia matrix in task space
        Mq_inv = np.linalg.inv(Mq)
        JEE_Mq_inv = np.dot(JEE, Mq_inv)  # save for use again below
        Mx_inv = np.dot(JEE_Mq_inv, JEE.T)
        # using the rcond to set singular values < thresh to 0
        # is slightly faster than doing it manually with svd
        Mx = np.linalg.pinv(Mx_inv, rcond=.01)

        # calculate desired force in (x,y,z) space
        if self.vmax is not None:
            # implement velocity limiting
            x_tilde = xyz - target_x
            sat = self.vmax / (self.lamb * np.abs(x_tilde))
            if np.any(sat < 1):
                index = np.argmin(sat)
                unclipped = self.kp * x_tilde[index]
                clipped = self.kv * self.vmax * np.sign(x_tilde[index])
                scale = np.ones(3, dtype='float32') * clipped / unclipped
                scale[index] = 1
            else:
                scale = np.ones(3, dtype='float32')

            u_xyz = -self.kv * (dx - target_dx -
                                np.clip(sat / scale, 0, 1) *
                                -self.lamb * scale * x_tilde)
        else:
            # generate (x,y,z) force without velocity limiting)
            u_xyz = (self.kp * (target_x - xyz)  +
                     self.kv * (target_dx - dx))
        print('u_xyz: ', u_xyz)

        u_xyz = np.dot(Mx, u_xyz)

        # if target_w is not None:
        #     # if a target orientation was provided, generate control
        #     # signal for the orientation of the ref_frame
        #     target_dw = np.zeros(3) if target_dw is None else target_dw
        #
        #     # convert target_w into a quaternion
        #     print('target_w: ', target_w)
        #     target_quat = (
        #         abr_control.utils.transformations.quaternion_from_euler(
        #             target_w[0], target_w[1], target_w[2], axes=w_axes))
        #
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

        print('Mq_g: ', Mq_g)

        # TODO: This is really awkward, but how else to get out
        # this signal for dynamics adaptation training?
        self.training_signal = np.dot(JEE.T, u_xyz)
        # add in gravity compensation, not included in training signal
        u = self.training_signal - Mq_g

        # NOTE: Should the null space controller be separated out
        # as a signal to be added in if chosen? -----------------

        # calculate the null space filter
        nkp = self.kp * .1
        nkv = np.sqrt(nkp)
        Jdyn_inv = np.dot(Mx, JEE_Mq_inv)
        null_filter = (np.eye(self.robot_config.num_joints) -
                       np.dot(JEE.T, Jdyn_inv))

        q_des = np.zeros(self.robot_config.num_joints, dtype='float32')
        dq_des = np.zeros(self.robot_config.num_joints, dtype='float32')

        # calculated desired joint angle acceleration using rest angles
        for ii in range(1, self.robot_config.num_joints):
            if self.robot_config.rest_angles[ii] is not None:
                q_des[ii] = (
                    ((self.robot_config.rest_angles[ii] - q[ii]) + np.pi) %
                     (np.pi*2) - np.pi)
                dq_des[ii] = dq[ii]
        u_null = np.dot(Mq, (nkp * q_des - nkv * dq_des))

        u += np.dot(null_filter, u_null)

        return u
