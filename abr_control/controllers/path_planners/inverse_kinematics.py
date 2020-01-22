import matplotlib.pyplot as plt
import numpy as np

from abr_control.utils import transformations


class InverseKinematics:
    """
    PARAMETERS
    ----------
    robot_config: class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    max_dx: float, Optional (Default: 0.2)
        the step size [meters] to take each step
    max_dr: float, Optional (Default: 2Pi)
        the step size [radians] to take each step
    max_dq: float, Optional (Default: Pi)
        the speed [rad/sec] to maintain each step
    """

    def __init__(self, robot_config, max_dx=0.2, max_dr=2 * np.pi, max_dq=np.pi):
        self.robot_config = robot_config
        self.max_dx = max_dx
        self.max_dr = max_dr
        self.max_dq = max_dq

    def generate_path(
        self,
        position,
        target_position,
        n_timesteps=200,
        dt=0.001,
        plot=False,
        method=3,
        axes="rxyz",
    ):
        """

        Parameters
        ----------
        position: numpy.array
            the current position of the system
        target_position: numpy.array
            the task space target position and orientation
        n_timesteps: int, optional (Default: 200)
            the number of time steps to reach the target_position
        dt: float, optional (Default: 0.001)
            the time step for calculating desired velocities [seconds]
        plot: boolean, optional (Default: False)
            plot the path after generating if True
        method: int, optional (Default: 3)
            Different ways to compute inverse resolved motion
            1. Standard resolved motion
            2. Dampened least squares method
            3. Nullspace with priority for position, orientation in null space
        axes: string, optional (Default: 'rxyz')
            the format of the Euler angles passed in target[3:6]
            First letter r or s represents 'relative' or 'static'
        """

        path = np.zeros((n_timesteps, position.shape[0] * 2))
        ee_track = []
        ee_err = []
        ea_err = []

        # set the largest allowable step in joint position
        max_dq = self.max_dq * dt
        # set the largest allowable step in hand (x,y,z)
        max_dx = self.max_dx * dt
        # set the largest allowable step in hand (alpha, beta, gamma)
        max_dr = self.max_dr * dt

        Qd = np.array(
            transformations.unit_vector(
                transformations.quaternion_from_euler(
                    target_position[3],
                    target_position[4],
                    target_position[5],
                    axes="sxyz",
                )
            )
        )

        q = np.copy(position)
        for ii in range(n_timesteps):
            J = self.robot_config.J("EE", q=q)
            T = self.robot_config.T("EE", q=q)
            ee_track.append(T[:3, 3])

            dx = target_position[:3] - T[:3, 3]

            Qe = self.robot_config.quaternion("EE", q=q)
            # Method 4
            dr = Qe[0] * Qd[1:] - Qd[0] * Qe[1:] - np.cross(Qd[1:], Qe[1:])

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
            pinv_Jx = np.linalg.pinv(Jx)

            # Different ways to compute inverse resolved motion
            if method == 1:
                # Standard resolved motion
                dq = np.dot(np.linalg.pinv(J), np.hstack([dx, dr]))
            if method == 2:
                # Dampened least squares method
                dq = np.dot(
                    J.T,
                    np.linalg.solve(
                        np.dot(J, J.T) + np.eye(6) * 0.001, np.hstack([dx, dr * 0.3])
                    ),
                )
            if method == 3:
                # Primary position IK, control orientation in null space
                dq = np.dot(pinv_Jx, dx) + np.dot(
                    np.eye(self.robot_config.N_JOINTS) - np.dot(pinv_Jx, Jx),
                    np.dot(np.linalg.pinv(J[3:]), dr),
                )

            # limit max step size in joint space
            if max(abs(dq)) > max_dq:
                dq = dq / max(abs(dq)) * max_dq

            path[ii] = np.hstack([q, dq])
            q = q + dq

        if plot:
            ee_track = np.array(ee_track)

            plt.subplot(2, 1, 1)
            plt.plot(ee_track)
            plt.gca().set_prop_cycle(None)
            plt.plot(np.ones((n_timesteps, 3)) * target_position[:3], "--")
            plt.legend(
                ["%i" % ii for ii in range(3)] + ["%i_target" % ii for ii in range(3)]
            )
            plt.title("Trajectory positions")

            plt.subplot(2, 1, 2)
            plt.plot(ee_err)
            plt.plot(ea_err)
            plt.legend(["Position error", "Orientation error"])
            plt.title("Trajectory orientations")

            plt.tight_layout()

            plt.show()
            plt.savefig("IK_plot.png")

        # reset position_path index
        self.n_timesteps = n_timesteps
        self.n = 0
        self.position_path = path[:, : self.robot_config.N_JOINTS]
        self.velocity_path = path[:, self.robot_config.N_JOINTS :]

        return self.position_path, self.velocity_path

    def next(self):
        """ Return the next target point along the generated position_path """

        # get the next target state if we're not at the end of the position_path
        self.position = (
            self.position_path[self.n] if self.n < self.n_timesteps else self.target
        )

        self.velocity = (
            self.position_path[self.n] if self.n < self.n_timesteps else self.velocity
        )
        self.n = min(self.n + 1, self.n_timesteps)

        return self.position, self.velocity
