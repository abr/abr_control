import numpy as np

from abr_control.utils import transformations

class InverseKinematics():

    def __init__(self, robot_config, max_dx=0.2, max_dr=2*np.pi, max_dq=np.pi):
        super(InverseKinematics, self).__init__(robot_config)
        self.max_dx = max_dx
        self.max_dr = max_dr
        self.max_dq = max_dq


    def generate_path(self, state, target, n_timesteps=200,
                      dt=0.001, plot=False, method=3):
        """

        Parameters
        ----------
        state : numpy.array
            the current position of the system
        target : numpy.array
            the target position and orientation
        n_timesteps : int, optional (Default: 200)
            the number of time steps to reach the target
        dt : float, optional (Default: 0.001)
            the time step for calculating desired velocities [seconds]
        plot : boolean, optional (Default: False)
            plot the path after generating if True
        method : int
            Different ways to compute inverse resolved motion
            1. Standard resolved motion
            2. Dampened least squares method
            3. Nullspace with priority for position, orientation in null space
        """

        self.trajectory = np.zeros((n_timesteps, state.shape[0]*2))
        ee_track = []
        ee_err = []
        ea_err = []

        # set the largest allowable step in joint position
        max_dq = self.max_dq * dt
        # set the largest allowable step in hand (x,y,z)
        max_dx = self.max_dx * dt
        # set the largest allowable step in hand (alpha, beta, gamma)
        max_dr = self.max_dr * dt

        Qd = np.array(transformations.unit_vector(
                transformations.quaternion_from_euler(
                    target[3], target[4], target[5], axes='rzyx')))

        q = np.copy(state)
        for ii in range(n_timesteps):
            J = self.robot_config.J('EE', q=q)
            T = self.robot_config.T('EE', q=q)
            ee_track.append(T[:3, 3])

            dx = target[:3] - T[:3, 3]

            Re = T[:3, :3]
            Qe = transformations.unit_vector(
                transformations.quaternion_from_matrix(Re))
            # Method 4
            dr = (Qe[0] * Qd[1:] -
                  Qd[0] * Qe[1:] -
                  np.cross(Qd[1:], Qe[1:]))

            norm_dx = np.linalg.norm(dx, 2)
            norm_dr = np.linalg.norm(dr, 2)
            ee_err.append(norm_dx)
            ea_err.append(norm_dr)

            # limit max step size in operational space
            if norm_dx > max_dx:
                dx = dx / norm_dx * max_dx
            if norm_dr > max_dr:
                dr = dr / norm_dr * max_dr

            Jx = J[:3]
            pinv_Jx = np.linalg.pinv(Jx)#, rcond=0.1)

            # Different ways to compute inverse resolved motion
            if method == 1:
                # Standard resolved motion
                dq = np.dot(np.linalg.pinv(J), np.hstack([dx, dr]))
            if method == 2:
                # Dampened least squares method
                dq = np.dot(
                    J.T,
                    np.linalg.solve(np.dot(J, J.T) + np.eye(6)*0.001,
                                    np.hstack([dx, dr*0.3])))
            if method == 3:
                # Primary position IK, control orientation in null space
                dq = np.dot(pinv_Jx, dx) + np.dot(
                    np.eye(self.robot_config.N_JOINTS) - np.dot(pinv_Jx, Jx),
                    np.dot(np.linalg.pinv(J[3:]), dr))

            # limit max step size in joint space
            if max(abs(dq)) > max_dq:
                dq = dq / max(abs(dq)) * max_dq

            # NOTE: should this be before or after q is updated?
            # I think after because it's setting the desired state for
            # the current time step, you want the velocity to be set to
            # take you to the next time step.
            self.trajectory[ii] = np.hstack([q, dq])

            q = q + dq


        if plot:
            import matplotlib.pyplot as plt
            ee_track = np.array(ee_track)

            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(ee_track)
            plt.gca().set_prop_cycle(None)
            plt.plot(np.ones((n_timesteps, 3)) * target[:3], '--')
            plt.legend(['%i' % ii for ii in range(3)] +
                       ['%i_target' % ii for ii in range(3)])
            plt.title('Trajectory positions')

            plt.subplot(2, 1, 2)
            plt.plot(ee_err)
            plt.plot(ea_err)
            plt.legend(['Position error', 'Orientation error'])
            plt.title('Trajectory orientations')

            plt.tight_layout()

            plt.show()
            plt.savefig('IK_plot.png')

            np.savez_compressed('config',
                q=self.trajectory[:, :6],
                time=np.arange(0, n_timesteps*dt, dt))

        # reset trajectory index
        self.n_timesteps = n_timesteps
        self.n = 0

        return (self.trajectory[:, :self.robot_config.N_JOINTS],
                self.trajectory[:, :self.robot_config.N_JOINTS],
                ee_err, ea_err)

    def next_target(self):
        """ Return the next target point along the generated trajectory """

        # get the next target state if we're not at the end of the trajectory
        self.target = (self.trajectory[self.n]
                       if self.n < self.n_timesteps else self.target)
        self.n += 1

        return self.target


if __name__ == '__main__':

    from abr_control.arms import ur5
    config = Config(use_cython=True)

    q = np.array([0, 45, -90, 45, 90, 115]) * np.pi / 180.0

    R = config.R('EE', q=q)

    targets = [
        np.array([ 0.0684952 , -0.58422071,  0.42894482, 1.57, 0.11, -2.7]),  # object orientation
        np.array([ 0.0684952 , -0.58422071,  0.22894482, 1.57, 0.11, -2.7]),  # object orientation
        np.array([ 0.0684952 , -0.58422071,  0.42894482, 1.57, 0.11, -2.7]),  # object orientation
        np.array([-0.62500012, -0.2249999 ,  0.30999999, 1.57, -1.22, -2.7]),  # deposit orientation
        np.array([-0.62500012, -0.2249999 ,  0.25999999, 1.57, -1.22, -2.7]),  # deposit orientation
        np.array([-0.62500012, -0.2249999 ,  0.40999999, 1.57, -1.22, -2.7]),  # deposit orientation
        np.array([-0.108958  , -0.22669163,  0.56309975, 1.57, 0.0, -2.7]),  # hand orientation
    ]

    ik = InverseKinematics(config)
    q_total = []
    ee_err_total = []
    ea_err_total = []
    n_timesteps = 5000
    dt = 0.001
    for target in targets:
        print('target: ', target)
        qs, dqs, ee_errs, ea_errs = ik.generate_path(
            state=q, target=target, method=1, dt=dt, n_timesteps=n_timesteps)
        q_total.append(qs)
        ee_err_total.append(ee_errs)
        ea_err_total.append(ea_errs)
        q = qs[-1]
    q_total = np.vstack(q_total)
    ee_err_total = np.hstack(ee_err_total)
    ea_err_total = np.hstack(ea_err_total)

    print(q_total.shape)

    np.savez_compressed(
        'config',
        q=q_total,
        time=np.arange(0, n_timesteps*dt*len(targets), dt),
        ee_err=ee_err_total,
        ea_err=ea_err_total)

    import matplotlib.pyplot as plt
    plt.plot(ee_err_total.T)
    plt.plot(ea_err_total.T)
    plt.show()
