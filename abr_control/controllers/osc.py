import numpy as np

from abr_control.utils import transformations

from .controller import Controller

class OSC(Controller):
    """ Implements an operational space controller (OSC)

    Parameters
    ----------
    robot_config: class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    kp: float, optional (Default: 1)
        proportional gain term on position error
    ko: float, optional (Default: same as kp)
        proportional gain term on orientation error
    kv: float, optional (Default: None)
        derivative gain term, a good starting point is sqrt(kp)
    ki: float, optional (Default: 0)
        integral gain term
    vmax: float, optional (Default: None)
        The max allowed task space xyz and orientation velocities [m/s, rad/s].
    ctrlr_dof: 6D list of booleans, optional (Default: position control)
        specifies which task space degrees of freedom are to be controlled
        [x, y, z, alpha, beta, gamma]
        NOTE: if more ctrlr_dof are specified than degrees of freedom in the
        robotic system, the controller will perform poorly
    null_controler: Controller, optional (Default: None)
        A controller to generate a secondary control signal to be
        applied in the null space of the OSC signal (i.e. applied as much
        as possible without affecting the movement of the end-effector)
    use_g: boolean, optional (Default: True)
        calculate and compensate for the effects of gravity
    use_C: boolean, optional (Default: False)
        calculate and compensate for the Coriolis and
        centripetal effects of the arm
    orientation_algorithm: int, optional (Default: 0)
        specify which orientation algorithm to use to calculate the task-space
        orientation forces to apply

    Attributes
    ----------
    integrated_error: float list, optional (Default: None)
        task-space integrated error term
    """
    def __init__(self, robot_config, kp=1, ko=None, kv=None, ki=0,
                 vmax=None, ctrlr_dof=None, null_controllers=None,
                 use_g=True, use_C=False, orientation_algorithm=0):

        super(OSC, self).__init__(robot_config)

        self.kp = kp
        self.ko = kp if ko is None else ko
        # TODO: find the appropriate default critical damping value
        # when using different position and orientation gains
        self.kv = np.sqrt(self.kp+self.ko) if kv is None else kv
        self.ki = ki
        self.null_controllers = null_controllers
        self.use_g = use_g
        self.use_C = use_C
        self.orientation_algorithm = orientation_algorithm

        if self.ki != 0:
            self.integrated_error = np.zeros(6)

        if ctrlr_dof is None:
            ctrlr_dof = [True, True, True, False, False, False]
        self.ctrlr_dof = np.copy(ctrlr_dof)
        self.n_ctrlr_dof = np.sum(self.ctrlr_dof)

        self.task_space_gains = np.array([self.kp,]*3 + [self.ko,]*3)
        self.lamb = self.task_space_gains / self.kv

        if self.n_ctrlr_dof > robot_config.N_JOINTS:
            print('\nRobot has fewer DOF (%i) than the specified number of ' %
                  robot_config.N_JOINTS + 'space dimensions to control (%i). ' %
                  self.n_ctrlr_dof + 'Poor performance may result.\n')

        self.vmax = vmax
        if vmax is not None:
            # precalculate gains used in velocity limiting
            self.sat_gain_xyz = vmax[0] / self.kp * self.kv
            self.sat_gain_abg = vmax[1] / self.ko * self.kv
            self.scale_xyz = vmax[0] / self.kp * self.kv
            self.scale_abg = vmax[1] / self.ko * self.kv

        self.ZEROS_SIX = np.zeros(6)
        self.IDENTITY_N_JOINTS = np.eye(self.robot_config.N_JOINTS)


    def _Mx(self, M, J, threshold=1e-3):
        """ Generate the task-space inertia matrix

        Parameters
        ----------
        M: np.array
            the generalized coordinates inertia matrix
        J: np.array
            the task space Jacobian
        threshold: scalar, optional (Default: 1e-3)
            singular value threshold, if the detminant of Mx_inv is less than
            this value then Mx is calculated using the pseudo-inverse function
            and all singular values < threshold * .1 are set = 0
        """

        # calculate the inertia matrix in task space
        M_inv = np.linalg.inv(M)
        Mx_inv = np.dot(J, np.dot(M_inv, J.T))
        if abs(np.linalg.det(Mx_inv)) >= threshold:
            # do the linalg inverse if matrix is non-singular
            # because it's faster and more accurate
            Mx = np.linalg.inv(Mx_inv)
        else:
            # using the rcond to set singular values < thresh to 0
            # singular values < (rcond * max(singular_values)) set to 0
            Mx = np.linalg.pinv(Mx_inv, rcond=threshold*.1)

        return Mx, M_inv


    def _calc_orientation_forces(self, target_abg, q):
        """ Calculate the desired Euler angle forces to apply to the arm to
        move the end-effector to the target orientation

        Parameters
        ----------
        target_abg: np.array
            the target Euler angles orientation for the end-effector:
            alpha, beta, gamma
        q: np.array
            the joint angles of the arm
        """
        u_task_orientation = np.zeros(3)
        if self.orientation_algorithm == 0:
            # transform the orientation target into a quaternion
            q_d = transformations.unit_vector(
                transformations.quaternion_from_euler(
                    target_abg[0], target_abg[1], target_abg[2], axes='rxyz'))
            # get the quaternion for the end effector
            q_e = self.robot_config.quaternion('EE', q=q)
            q_r = transformations.quaternion_multiply(
                q_d, transformations.quaternion_conjugate(q_e))
            u_task_orientation = -q_r[1:] * np.sign(q_r[0])

        elif self.orientation_algorithm == 1:
            # From (Caccavale et al, 1997) Section IV Quaternion feedback
            # get rotation matrix for the end effector orientation
            R_e = self.robot_config.R('EE', q)
            # get rotation matrix for the target orientation
            R_d = transformations.euler_matrix(
                target_abg[0], target_abg[1], target_abg[2],
                axes='rxyz')[:3, :3]
            R_ed = np.dot(R_e.T, R_d)  # eq 24
            q_ed = transformations.unit_vector(
                transformations.quaternion_from_matrix(R_ed))
            u_task_orientation = -1 * np.dot(R_e, q_ed[1:])  # eq 34

        else:
            raise Exception('Invalid algorithm number %i for calculating ' %
                            self.orientation_algorithm+ 'orientation error')

        return u_task_orientation


    def _velocity_limiting(self, u_task):
        """ Scale the control signal such that the arm isn't driven to move
        faster in position or orientation than the specified vmax values

        Parameters
        ----------
        u_task: np.array
            the task space control signal
        """
        norm_xyz = np.linalg.norm(u_task[:3])
        norm_abg = np.linalg.norm(u_task[3:])
        scale = np.ones(6)
        if norm_xyz > self.sat_gain_xyz:
            scale[:3] *= self.scale_xyz / norm_xyz
        if norm_abg > self.sat_gain_abg:
            scale[3:] *= self.scale_abg / norm_abg

        return self.kv * scale * self.lamb * u_task


    def generate(self, q, dq, target, target_vel=None,
                 ref_frame='EE', xyz_offset=None):
        """ Generates the control signal to move the EE to a target

        Parameters
        ----------
        q: float numpy.array
            current joint angles [radians]
        dq: float numpy.array
            current joint velocities [radians/second]
        target: 6 dimensional float numpy.array
            desired task space position and orientation [meters, radians]
            orientation component is alpha, beta, gamma in relative xyz axes
        target_vel: float 6D numpy.array, optional (Default: None)
            desired task space velocities [meters/sec, radians/sec]
        ref_frame: string, optional (Default: 'EE')
            the point being controlled, default is the end-effector.
        xyz_offset: list, optional (Default: None)
            point of interest inside the frame of reference [meters]
        """

        if target_vel is None:
            target_vel = self.ZEROS_SIX

        J = self.robot_config.J(ref_frame, q, x=xyz_offset)  # Jacobian
        # isolate rows of Jacobian corresponding to controlled task space DOF
        J = J[self.ctrlr_dof]

        M = self.robot_config.M(q)  # inertia matrix in joint space
        Mx, M_inv = self._Mx(M=M, J=J)  # inertia matrix in task space

        # calculate the desired task space forces -----------------------------
        u_task = np.zeros(6)

        # if position is being controlled
        if np.sum(self.ctrlr_dof[:3]) > 0:
            xyz = self.robot_config.Tx(ref_frame, q, x=xyz_offset)
            u_task[:3] = xyz - target[:3]

        # if orientation is being controlled
        if np.sum(self.ctrlr_dof[3:]) > 0:
            u_task[3:] = self._calc_orientation_forces(target[3:], q)

        # task space integrated error term
        if self.ki != 0:
            self.integrated_error += u_task
            u_task += self.ki * self.integrated_error

        u = np.zeros(self.robot_config.N_JOINTS)
        if self.vmax is not None:
            # if max task space velocities specified, apply velocity limiting
            u_task = self._velocity_limiting(u_task)
        else:
            # otherwise apply specified gains
            u_task *= self.task_space_gains

        # compensate for velocity
        if np.all(target_vel == 0):
            # if there's no target velocity in task space,
            # compensate for velocity in joint space (more accurate)
            u = -1 * self.kv * np.dot(M, dq)
        else:
            dx = np.zeros(6)
            dx[self.ctrlr_dof] = np.dot(J, dq)
            u_task += self.kv * (dx - target_vel)

        # isolate task space forces corresponding to controlled DOF
        u_task = u_task[self.ctrlr_dof]

        # transform task space control signal into joint space ----------------
        u -= np.dot(J.T, np.dot(Mx, u_task))

        # add in estimation of full centrifugal and Coriolis effects ----------
        if self.use_C:
            u -= np.dot(self.robot_config.C(q=q, dq=dq), dq)

        # store the current control signal u for training in case
        # dynamics adaptation signal is being used
        # NOTE: do not include gravity or null controller in training signal
        self.training_signal = np.copy(u)

        # add in gravity term in joint space ----------------------------------
        if self.use_g:
            u -= self.robot_config.g(q=q)

            # add in gravity term in task space
            # Jbar = np.dot(M_inv, np.dot(J.T, Mx))
            # g = self.robot_config.g(q=q)
            # self.u_g = g
            # g_task = np.dot(Jbar.T, g)

        # add in secondary control signals ------------------------------------
        if self.null_controllers is not None:
            for null_controller in self.null_controllers:
                # generate control signal to apply in null space
                u_null = null_controller.generate(q, dq)
                # calculate null space filter
                Jbar = np.dot(M_inv, np.dot(J.T, Mx))
                null_filter = (self.IDENTITY_N_JOINTS - np.dot(J.T, Jbar.T))
                # add in filtered null space control signal
                u += np.dot(null_filter, u_null)

        return u
