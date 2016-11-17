import numpy as np


class controller:
    """ Implements an operational space controller (OSC)
    """

    def __init__(self, robot_config, kp=100, kv=None,
                 vmax=0.5, obstacles=[], threshold=.2):

        self.robot_config = robot_config

        # proportional gain term
        self.kp = kp
        # derivative gain term
        self.kv = np.sqrt(self.kp) if kv is None else kv
        # velocity limit of the end-effector
        self.vmax = vmax
        # set of obstacles [[x, y, radius], ...]
        self.obstacles = obstacles
        self.threshold = threshold

    def control(self, q, dq, target_state):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_state np.array: the target [pos, vel] for the end-effector
        """

        # calculate position of the end-effector
        xyz = self.robot_config.Tx('EE', q)

        # calculate the Jacobian for the end effector
        JEE = self.robot_config.J('EE', q)

        # calculate the inertia matrix in joint space
        Mq = self.robot_config.Mq(q)

        # calculate the effect of gravity in joint space
        Mq_g = self.robot_config.Mq_g(q)

        # convert the mass compensation into end effector space
        Mx_inv = np.dot(JEE, np.dot(np.linalg.pinv(Mq), JEE.T))
        svd_u, svd_s, svd_v = np.linalg.svd(Mx_inv)
        # cut off any singular values that could cause control problems
        singularity_thresh = .00025
        for ii in range(len(svd_s)):
            svd_s[ii] = 0 if svd_s[ii] < singularity_thresh else \
                1./float(svd_s[ii])
        # numpy returns U,S,V.T, so have to transpose both here
        Mx = np.dot(svd_v.T, np.dot(np.diag(svd_s), svd_u.T))

        # calculate desired force in (x,y,z) space
        dx = np.dot(JEE, dq)
        # implement velocity limiting
        lamb = self.kp / self.kv
        x_tilde = xyz - target_state[:3]
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
        u_xyz = np.dot(Mx, u_xyz)

        self.training_signal = np.dot(JEE.T, u_xyz)
        # add in gravity compensation, not included in training signal
        u = self.training_signal - Mq_g

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
        nkp = self.kp * .1
        nkv = np.sqrt(nkp)
        u_null = np.dot(Mq, (nkp * q_des - nkv * dq_des))

        u += np.dot(null_filter, u_null)

        # add in obstacles avoidance
        for obstacle in self.obstacles:
            # our vertex of interest is the center point of the obstacle
            v = np.array(obstacle[:3])

            # find the closest point of each link to the obstacle
            for ii in range(self.robot_config.num_links):
                # get the start and end-points of the arm segment
                p1 = self.robot_config.Tx('joint%i' % ii, q=q)
                if ii == self.robot_config.num_links - 1:
                    p2 = self.robot_config.Tx('EE', q=q)
                else:
                    p2 = self.robot_config.Tx('joint%i' % (ii + 1), q=q)

                # calculate minimum distance from arm segment to obstacle
                # the vector of our line
                vec_line = p2 - p1
                # the vector from the obstacle to the first line point
                vec_ob_line = v - p1
                # calculate the normalized angle between these two lines
                angle = np.dot(vec_ob_line, vec_line) / np.sum((vec_line)**2)
                if angle < 0:
                    # then closest point is the start of the segment
                    closest = p1
                elif angle > 1:
                    # then closest point is the end of the segment
                    closest = p2
                else:
                    closest = p1 + angle * vec_line
                # calculate distance from obstacle vertex to the closest point
                dist = np.sqrt(np.sum((v - closest)**2))
                # account for size of obstacle
                ro = dist - obstacle[3]

                if ro < self.threshold:

                    eta = .2  # feel like i saw 4 somewhere in the paper
                    drodx = (v - closest) / ro
                    Fpsp = (eta * (1.0/ro - 1.0/self.threshold) *
                            1.0/ro**2 * drodx)

                    # get offset of closest point from link's reference frame
                    T_inv = self.robot_config.T_inv('link%i' % ii, q=q)
                    m = np.dot(T_inv, np.hstack([closest, [1]]))[:-1]
                    # calculate the Jacobian for this point
                    Jpsp = self.robot_config.J('link%i' % ii, x=m, q=q)[:3]

                    # calculate the inertia matrix for the
                    # point subjected to the potential space
                    Mxpsp_inv = np.dot(Jpsp,
                                       np.dot(np.linalg.pinv(Mq), Jpsp.T))
                    svd_u, svd_s, svd_v = np.linalg.svd(Mxpsp_inv)
                    # cut off singular values that could cause problems
                    singularity_thresh = .00025
                    for ii in range(len(svd_s)):
                        svd_s[ii] = 0 if svd_s[ii] < singularity_thresh else \
                            1./float(svd_s[ii])
                    # numpy returns U,S,V.T, so have to transpose both here
                    Mxpsp = np.dot(svd_v.T, np.dot(np.diag(svd_s), svd_u.T))

                    u_psp = -np.dot(Jpsp.T, np.dot(Mxpsp, Fpsp))
                    if ro < .01:
                        u = u_psp
                    else:
                        u += u_psp

        return u
