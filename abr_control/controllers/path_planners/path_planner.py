import math
import warnings

import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate
from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611

from abr_control.controllers.path_planners.orientation import Orientation
from abr_control.utils import colors as c
from abr_control.utils import transformations as transform


class PathPlanner:
    def __init__(self, pos_profile, vel_profile, axes="rxyz", verbose=False):
        """
        Generalized path planner that outputs a velocity limited path
        - Takes a position profile and velocity profile to define the shape and speed
            - position profile is a function that outputs a 3D value in the domain of [0, 1]
                - at t==0 the profile must be [0, 0, 0]
                - at t==1 the profile must be [1, 1, 1]
            - velocity profile is a function that outputs a 1D list of velocities from
            a start to a target velocity given some time step dt
        - the generate_path function will warp the position profile so that it starts and
        ends at the defined location, while maintaining the velocity profile. The velocity
        profile will be limited to max_velocity. Once the max velocity is reached it will
        be maintained until it is time to decelrate to the target velocity (wrt the vel_profile).
        The final velocity profile will go from start_velocity to max_velocity, and back down
        the target_velocity. If we do not have enough time to reach max_velocity we will
        accelerate until it is time to decelerate to target_velocity.

        - A start and target orientation can optionally be passed in to generate_path
        - the order of the euler angles is defined by 'axes' on init
        - quaternion slerp is used to smoothly transition from start to target orientation,
        following the velocity profile so that we reach the target orientation at the same
        moment we reach out target position and target velocity.

        Parameters
        ----------
        pos_profile: position_profiles class
            must have a step function that takes in a float from 0 to 1, and returns a
            3x1 array. This defines the shape of the desired path, with t(0) defining
            the starting position at [0, 0, 0], and t(1) defining the target at [1, 1, 1].
            The path planner will do the appropriate warping to the actual start and target.
        vel_profile: velocity profiles class
            must accept dt on init.
            must have a generate function that takes in start and target velocities as floats, and
            returns a 1xN list that transitions between them, where N is determined by dt.
        axes: string, Optional (Default: 'rxyz')
            The euler order of state and target orientations
        verbose: bool, Optional (Default: False)
            True for extra debug prints
        """
        self.n_sample_points = pos_profile.n_sample_points
        self.dt = vel_profile.dt
        self.pos_profile = pos_profile
        self.vel_profile = vel_profile
        self.axes = axes
        self.OrientationPlanner = Orientation(axes=self.axes)
        self.n = 0
        self.n_timesteps = None
        self.target_counter = 0
        self.verbose = verbose
        self.log = []

        self.starting_vel_profile = None
        self.ending_vel_profile = None
        self.start_velocity = 0  # can be overwritten by generate_path
        self.target_velocity = 0  # can be overwritten by generate_path
        self.path = np.zeros((12, 1))

    def align_vectors(self, a, b):
        """
        Takes vectors a and b and returns rotation matrix to align a to b

        Parameters
        ----------
        a: 3x1 array of floats
            vector we are rotating
        b: 3xa array of floats
            vector we are trying to align to
        """
        b = b / np.linalg.norm(b)  # normalize a
        a = a / np.linalg.norm(a)  # normalize b
        v = np.cross(a, b)
        c = np.dot(a, b)

        v1, v2, v3 = v
        h = 1 / (1 + c)

        Vmat = np.array([[0, -v3, v2], [v3, 0, -v1], [-v2, v1, 0]])

        R = np.eye(3, dtype=np.float64) + Vmat + (Vmat.dot(Vmat) * h)
        return R

    def generate_path(
        self,
        start_position,
        target_position,
        max_velocity,
        start_orientation=None,
        target_orientation=None,
        start_velocity=0,
        target_velocity=0,
        plot=False,
    ):
        """
        Takes a start and target position, along with an optional start and target velocity,
        and generates a trajectory that smoothly accelerates, at a rate defined by vel_profile,
        from start_velocity to max_v, and back to target_v. If the path is too short to reach max_v
        and still decelerate to target_v at a rate of max_a, then the path will be slowed to
        the maximum allowable velocity so that we still reach target_velocity at the moment we are at
        target_position.
        Optionally can pass in a 3D angular state [a, b, g] and target orientation. Note that
        the orientation should be given in euler angles, in the ordered specified by axes on init.
        The orientation path will follow the same velocity profile as the position path.

        Parameters
        ----------
        start_position: 3x1 np.array of floats
            starting position
        target_position: 3x1 np.array of floats
            target position
        max_velocity: float
            the maximum allowable velocity of the path
        start_velocity: float, Optional (Default: 0)
            velocity at start of path
        target_velocity: float, Optional (Default: 0)
            velocity at end of path
        start_orientation: 3x1 np.array of floats, Optional (Default: None)
            orientation at start of path in euler angles, given in the order specified
            on __init__ with the axes parameter (default rxyz).
            When left as None no orientation path will be planned
        target_orientation: 3x1 np.array of floats, Optional (Default: None)
            the target orientation at the end of the path in euler angles, given in the
            order specified on __init__ with the axes parameter (default rxzyz)
        plot: bool, Optional (Default: False)
            True to plot path profiles for debugging
        """
        assert start_velocity <= max_velocity, (
            f"{c.red}start velocity({start_velocity}m/s) "
            + f"> max velocity({max_velocity}m/s){c.endc}"
        )
        assert target_velocity <= max_velocity, (
            f"{c.red}target velocity({target_velocity}m/s) "
            + f"> max velocity({max_velocity}m/s){c.endc}"
        )

        if start_velocity == max_velocity:
            self.starting_dist = 0
            self.starting_vel_profile = [start_velocity * self.dt]
        else:
            self.starting_dist = None

        if target_velocity == max_velocity:
            self.ending_dist = 0
            self.ending_vel_profile = [target_velocity * self.dt]
        else:
            self.ending_dist = None

        self.max_velocity = max_velocity

        if self.starting_dist is None:
            # Regenerate our velocity curves if start or end v have changed
            if (
                self.starting_vel_profile is None
                or self.start_velocity != start_velocity
            ):
                self.starting_vel_profile = self.vel_profile.generate(
                    start_velocity=start_velocity, target_velocity=self.max_velocity
                )

            # calculate the distance covered ramping from start_velocity to max_v
            # and from max_v to target_velocity
            self.starting_dist = np.sum(self.starting_vel_profile * self.dt)

        if self.ending_dist is None:
            # if our start and end v are the same, just mirror the curve to avoid regenerating
            if start_velocity == target_velocity:
                self.ending_vel_profile = self.starting_vel_profile[::-1]

            # if target velocity is different, generate its unique curve
            elif (
                self.ending_vel_profile is None
                or self.target_velocity != target_velocity
            ):
                self.ending_vel_profile = self.vel_profile.generate(
                    start_velocity=target_velocity, target_velocity=self.max_velocity
                )[::-1]

            # calculate the distance covered ramping from start_velocity to max_v
            # and from max_v to target_velocity
            self.ending_dist = np.sum(self.ending_vel_profile * self.dt)

        # save as self variables so we can check on the next generate call if we need a different profile
        self.start_velocity = start_velocity
        self.target_velocity = target_velocity

        if self.verbose:
            self.log.append(
                f"{c.blue}Generating a path from {start_position} to {target_position}{c.endc}"
            )
            self.log.append(f"{c.blue}max_velocity={self.max_velocity}{c.endc}")
            self.log.append(
                f"{c.blue}start_velocity={self.start_velocity} | target_velocity={self.target_velocity}{c.endc}"
            )

        # calculate the distance between our current state and the target
        target_direction = target_position - start_position
        dist = np.linalg.norm(target_direction)
        target_norm = target_direction / dist

        # the default direction of our path shape
        a = 1 / np.sqrt(3)
        base_norm = np.array([a, a, a])
        # get the rotation matrix to rotate our path shape to the target direction
        R = self.align_vectors(base_norm, target_norm)

        # get the length travelled along our stretched curve
        curve_dist_steps = []
        warped_xyz = []
        for ii, t in enumerate(np.linspace(0, 1, self.n_sample_points)):
            # ==== Warp our path ====
            # - stretch our curve shape to be the same length as target-start
            # - rotate our curve to align with the direction target-start
            # - shift our curve to begin at the start state
            warped_target = (
                np.dot(R, (1 / np.sqrt(3)) * self.pos_profile.step(t) * dist)
                + start_position
            )
            warped_xyz.append(warped_target)
            if t > 0:
                curve_dist_steps.append(
                    np.linalg.norm(warped_xyz[ii] - warped_xyz[ii - 1])
                )
            else:
                curve_dist_steps.append(0)

        # get the cumulative distance covered
        dist_steps = np.cumsum(curve_dist_steps)
        curve_length = np.sum(curve_dist_steps)
        # create functions that return our path at a given distance
        # along that curve
        self.warped_xyz = np.array(warped_xyz)
        X = scipy.interpolate.interp1d(
            dist_steps, self.warped_xyz.T[0], fill_value="extrapolate"
        )
        Y = scipy.interpolate.interp1d(
            dist_steps, self.warped_xyz.T[1], fill_value="extrapolate"
        )
        Z = scipy.interpolate.interp1d(
            dist_steps, self.warped_xyz.T[2], fill_value="extrapolate"
        )
        XYZ = [X, Y, Z]

        # distance is greater than our ramping up and down distance
        # add a linear velocity from between the ramps to converge to the correct position
        self.remaining_dist = None
        if curve_length >= self.starting_dist + self.ending_dist:
            # calculate the remaining steps where we will be at constant max_v
            remaining_dist = curve_length - (self.ending_dist + self.starting_dist)
            constant_speed_steps = int(remaining_dist / self.max_velocity / self.dt)

            self.stacked_vel_profile = np.hstack(
                (
                    self.starting_vel_profile,
                    np.ones(constant_speed_steps) * self.max_velocity,
                    self.ending_vel_profile,
                )
            )
            if plot:
                self.remaining_dist = remaining_dist
                self.dist = dist
        else:
            # scale our profile
            # TODO to do this properly we should evaluate the integral to get the t where
            # the sum of the profile is half our travel distance. This way we maintain
            # the same acceleration profile instead of maintaining the same number of steps
            # and accelerating more slowly
            scale = curve_length / (self.starting_dist + self.ending_dist)
            self.stacked_vel_profile = np.hstack(
                (scale * self.starting_vel_profile, scale * self.ending_vel_profile)
            )

        self.position_path = []

        # the distance covered over time with respect to our velocity profile
        path_steps = np.cumsum(self.stacked_vel_profile * self.dt)
        # due to the interpolation and discretization, we may have errors in our distances
        # assure that we are not going passed our curve length to avoid errors interpolated
        # passed the interpolation range

        # path_steps[path_steps>curve_length] = curve_length

        # step along our curve, with our next path step being the
        # distance determined by our velocity profile, in the direction of the path curve
        for ii in range(0, len(self.stacked_vel_profile)):
            shiftx = XYZ[0](path_steps[ii])
            shifty = XYZ[1](path_steps[ii])
            shiftz = XYZ[2](path_steps[ii])
            shift = np.array([shiftx, shifty, shiftz])
            self.position_path.append(shift)

        self.position_path = np.asarray(self.position_path)

        # get our 3D vel profile components by differentiating the position path
        # Since we use our velocity profile to approximate the position path, differentiate
        # that path to get our velocity so that the two correlate more closely
        self.velocity_path = np.asarray(
            np.gradient(self.position_path, self.dt, axis=0)
        )

        # check if we received start and target orientations
        if isinstance(start_orientation, (list, (np.ndarray, np.generic), tuple)):
            if isinstance(target_orientation, (list, (np.ndarray, np.generic), tuple)):
                # Generate the orientation portion of our trajectory.
                # We will use quaternions and SLERP for filtering from start_quat to target_quat.
                quat0 = transform.quaternion_from_euler(
                    start_orientation[0],
                    start_orientation[1],
                    start_orientation[2],
                    axes=self.axes,
                )

                quat1 = transform.quaternion_from_euler(
                    target_orientation[0],
                    target_orientation[1],
                    target_orientation[2],
                    axes=self.axes,
                )

                self.orientation_path = self.OrientationPlanner.match_position_path(
                    orientation=quat0,
                    target_orientation=quat1,
                    position_path=self.position_path,
                )
                if self.verbose:
                    self.log.append(
                        f"{c.blue}start_orientation={start_orientation} | target_orientation={target_orientation}{c.endc}"
                    )
            else:
                raise NotImplementedError(
                    f"{c.red}A target orientation is required to generate a path{c.endc}"
                )

            self.orientation_path = np.asarray(self.orientation_path)
            # TODO should this be included? look at proper derivation here...
            # https://physics.stackexchange.com/questions/73961/angular-velocity-expressed-via-euler-angles
            self.ang_velocity_path = np.asarray(
                np.gradient(self.orientation_path, self.dt, axis=0)
            )

            self.path = np.hstack(
                (
                    np.hstack(
                        (
                            np.hstack((self.position_path, self.velocity_path)),
                            self.orientation_path,
                        )
                    ),
                    self.ang_velocity_path,
                )
            )
            self.angular_planner = True
            if plot:
                self._plot(
                    start_position=start_position,
                    target_position=target_position,
                    ang=True,
                )
        else:
            self.angular_planner = False
            self.path = np.hstack((self.position_path, self.velocity_path))
            if plot:
                self._plot(
                    start_position=start_position,
                    target_position=target_position,
                    ang=False,
                )

        # Some parameters that are useful to have access to externally, used in nengo-control
        self.n_timesteps = len(self.path)
        self.n = 0
        self.time_to_converge = self.n_timesteps * self.dt
        self.target_counter += 1

        if self.verbose:
            self.log.append(
                f"{c.blue}Time to converge: {self.time_to_converge}{c.endc}"
            )
            self.log.append(f"{c.blue}dt: {self.dt}{c.endc}")
            self.log.append(
                f"{c.blue}pos x error: {self.position_path[-1, 0] - target_position[0]}{c.endc}"
            )
            self.log.append(
                f"{c.blue}pos y error: {self.position_path[-1, 1] - target_position[1]}{c.endc}"
            )
            self.log.append(
                f"{c.blue}pos x error: {self.position_path[-1, 2] - target_position[2]}{c.endc}"
            )
            self.log.append(
                f"{c.blue}2norm error at target: {np.linalg.norm(self.position_path[-1] - target_position[:3])}{c.endc}"
            )

            dash = "".join((["-"] * len(max(self.log, key=len))))
            print(f"{c.blue}{dash}{c.endc}")
            for log in self.log:
                print(log)
            print(f"{c.blue}{dash}{c.endc}")

        err = np.linalg.norm(self.position_path[-1] - target_position)
        if err >= 0.01:
            warnings.warn(
                (
                    f"\n{c.yellow}WARNING: the distance at the end of the generated path to your "
                    + f"desired target position is {err}m. If you desire a lower error you can try:"
                    + f"\n\t- a path shape with lower frequency terms"
                    + f"\n\t- more sample points (set on __init__)"
                    + f"\n\t- smaller simulation timestep"
                    + f"\n\t- lower maximum velocity and acceleration"
                    + f"\n\t- lower start and end velocities{c.endc}"
                )
            )

        return self.path

    def next(self):
        """
        Returns the next target from the generated path
        """
        path = self.path[self.n]
        if self.n_timesteps is not None:
            self.n = min(self.n + 1, self.n_timesteps - 1)
        else:
            self.n += 1

        return path

    def convert_to_time(self, path, time_length):
        """Accepts a pregenerated path from current state to target and
        interpolates with respect to the time_limit. The function can
        then be stepped through to reach a target within the specified time.

        PARAMETERS
        ----------
        path: numpy.array
            The output from a subclasses generate_path() function
        time_length: float
            the desired time to go from state to target [seconds]
        """

        n_states = np.asarray(path).shape[1]

        # interpolate the function to the specified time_limit
        times = np.linspace(0, time_length, self.n_timesteps)
        path_func = []
        for dim in range(n_states):
            path_func.append(
                scipy.interpolate.interp1d(times, np.asarray(path)[:, dim])
            )

        return path_func

    def _plot(self, start_position, target_position, ang=None):
        """
        Only called internally if plot=True in the generate_path call
        Plots several profiles of the path for debugging. Most of the parameters accessed are
        saved as self variables.

        Parameters
        ----------
        start_position: 3x1 np.array of floats
            starting position
        target_position: 3x1 np.array of floats
            target position
        """
        len_start = len(self.starting_vel_profile)
        len_end = len(self.ending_vel_profile)
        plt.figure()
        if ang:
            cols = 2
        else:
            cols = 1

        # plot the components of the position path along with markers for the start and target
        ax_fillbetween = plt.subplot(2, cols, 1)
        plt.title("Position")
        steps = self.position_path.shape[0]
        plt.plot(self.position_path[:, 0], "r")
        plt.plot(self.position_path[:, 1], "b")
        plt.plot(self.position_path[:, 2], "g")

        plt.scatter(0, start_position[0], c="r")
        plt.scatter(0, start_position[1], c="b")
        plt.scatter(0, start_position[2], c="g")
        plt.scatter(steps, target_position[0], c="r")
        plt.scatter(steps, target_position[1], c="b")
        plt.scatter(steps, target_position[2], c="g")

        # add markers where our velocity ramps stop and end
        plt.plot(np.linalg.norm(self.position_path, axis=1))
        plt.scatter(len_start, self.starting_dist, c="m")
        if self.remaining_dist is not None:
            plt.scatter(
                steps - len_end, self.remaining_dist + self.starting_dist, c="c"
            )
            plt.scatter(
                steps,
                self.ending_dist + self.starting_dist + self.remaining_dist,
                c="k",
            )
        # fill the space signifying the linear portion of the path
        ax_fillbetween.axvspan(
            len_start, steps - len_end, alpha=0.25, label="linear velocity"
        )

        plt.legend(["x", "y", "z"])

        # plot the components of the velocity profile
        plt.subplot(2, cols, 2)
        plt.title("Velocity")
        plt.plot(self.velocity_path[:, 0], "r")
        plt.plot(self.velocity_path[:, 1], "b")
        plt.plot(self.velocity_path[:, 2], "g")

        # plot the normalized velocities and markers for the limits
        norm = []
        for vel in self.velocity_path:
            norm.append(np.linalg.norm(vel))
        plt.plot(norm, "y")
        plt.plot([self.max_velocity] * len(norm), linestyle="--")
        plt.plot([self.start_velocity] * len(norm), linestyle="--")
        plt.plot([self.target_velocity] * len(norm), linestyle="--")
        plt.legend(["dx", "dy", "dz", "norm", "vel limit", "start_vel", "target_vel"])

        # plot the orientation path if it exists
        if ang:
            plt.subplot(2, cols, 3)
            plt.title("Orientation")
            plt.plot(self.orientation_path)
            plt.legend(["a", "b", "g"])

            plt.subplot(2, cols, 4)
            plt.title("Angular Velocity")
            plt.plot(self.ang_velocity_path)
            plt.legend(["da", "db", "dg"])

        plt.tight_layout()

        # plots of given and warped position profile
        curve = []
        x = np.linspace(0, 1, self.n_sample_points)
        for xx in x:
            curve.append(self.pos_profile.step(xx))
        curve = np.array(curve).T

        # plot the shape of the given curve
        plt.figure()
        ax1 = plt.subplot(131, projection="3d")
        ax1.set_title("Given Curve")
        ax1.plot(curve[0], curve[1], curve[2])

        xerr = self.position_path[-1, 0] - target_position[0]
        yerr = self.position_path[-1, 1] - target_position[1]
        zerr = self.position_path[-1, 2] - target_position[2]
        dist_err = np.linalg.norm(self.position_path[-1] - target_position[:3])

        # plot the transformed curve
        # this is scaled, rotated, and shifted
        ax3 = plt.subplot(132, projection="3d")
        ax3.set_title("Warped Curve")
        ax3.plot(self.warped_xyz.T[0], self.warped_xyz.T[1], self.warped_xyz.T[2])
        ax3.scatter(
            start_position[0], start_position[1], start_position[2], label="start"
        )
        ax3.scatter(
            target_position[0], target_position[1], target_position[2], label="target"
        )
        ax3.legend()

        # plot the final path interpolated from the warped curve
        ax3 = plt.subplot(133, projection="3d")
        ax3.set_title("Interpolated Position Path")
        ax3.plot(
            self.position_path[:, 0],
            self.position_path[:, 1],
            self.position_path[:, 2],
            label=f"error at target={dist_err:.4f}m",
        )
        ax3.scatter(
            start_position[0], start_position[1], start_position[2], label="start"
        )
        ax3.scatter(
            target_position[0], target_position[1], target_position[2], label="target"
        )
        ax3.legend()

        # plot the components of the given curve
        plt.figure()
        plt.subplot(3, 3, 1)
        plt.title("Given X Shape")
        plt.xlabel("Steps [unitless]")
        plt.plot(curve[0])
        plt.subplot(3, 3, 2)
        plt.title("Given Y Shape")
        plt.xlabel("Steps [unitless]")
        plt.plot(curve[1])
        plt.subplot(3, 3, 3)
        plt.title("Given Z Shape")
        plt.xlabel("Steps [unitless]")
        plt.plot(curve[2])

        # plot the components of the warped curve
        plt.subplot(3, 3, 4)
        plt.title("Warped X Path")
        plt.xlabel("Steps [unitless]")
        plt.plot(self.warped_xyz.T[0])
        plt.hlines(
            start_position[0],
            0,
            self.n_sample_points - 1,
            linestyle="--",
            color="y",
            label="start",
        )
        plt.hlines(
            target_position[0],
            0,
            self.n_sample_points - 1,
            linestyle="--",
            color="g",
            label="target",
        )
        plt.legend()
        plt.subplot(3, 3, 5)
        plt.title("Warped Y Path")
        plt.xlabel("Steps [unitless]")
        plt.plot(self.warped_xyz.T[1])
        plt.hlines(
            start_position[1],
            0,
            self.n_sample_points - 1,
            linestyle="--",
            color="y",
            label="start",
        )
        plt.hlines(
            target_position[1],
            0,
            self.n_sample_points - 1,
            linestyle="--",
            color="g",
            label="target",
        )
        plt.legend()
        plt.subplot(3, 3, 6)
        plt.title("Warped Z Path")
        plt.xlabel("Steps [unitless]")
        plt.plot(self.warped_xyz.T[2])
        plt.hlines(
            start_position[2],
            0,
            self.n_sample_points - 1,
            linestyle="--",
            color="y",
            label="start",
        )
        plt.hlines(
            target_position[2],
            0,
            self.n_sample_points - 1,
            linestyle="--",
            color="g",
            label="target",
        )
        plt.legend()

        # plot the components of the path from the interpolated function
        t = np.arange(0, self.position_path.shape[0])
        t = self.dt * np.asarray(t)

        plt.subplot(3, 3, 7)
        plt.title("Interpolated X Path")
        plt.plot(t, self.position_path[:, 0], label=f"x_err={xerr:.4f}m")
        plt.hlines(
            start_position[0], 0, t[-1], linestyle="--", color="y", label="start"
        )
        plt.hlines(
            target_position[0], 0, t[-1], linestyle="--", color="g", label="target"
        )
        plt.xlabel("Time [sec]")
        plt.legend()
        plt.subplot(3, 3, 8)
        plt.title("Interpolated Y Path")
        plt.plot(t, self.position_path[:, 1], label=f"y_err={yerr:.4f}m")
        plt.hlines(
            start_position[1], 0, t[-1], linestyle="--", color="y", label="start"
        )
        plt.hlines(
            target_position[1], 0, t[-1], linestyle="--", color="g", label="target"
        )
        plt.xlabel("Time [sec]")
        plt.legend()
        plt.subplot(3, 3, 9)
        plt.title("Interpolated Z Path")
        plt.plot(t, self.position_path[:, 2], label=f"z_err={zerr:.4f}m")
        plt.hlines(
            start_position[2], 0, t[-1], linestyle="--", color="y", label="start"
        )
        plt.hlines(
            target_position[2], 0, t[-1], linestyle="--", color="g", label="target"
        )
        plt.xlabel("Time [sec]")
        plt.legend()
        plt.tight_layout()

        plt.show()
