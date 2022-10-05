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
        - Takes a position and velocity profile to define the shape and speed
            - Position profile is a function that outputs a 3D value
                - at t==0 the profile must be [0, 0, 0]
                - at t==1 the profile must be [1, 1, 1]
            - Velocity profile is a function that outputs a 1D list of velocities from
            a start to a target velocity given some time step dt

        - The generate_path function will warp the position profile so that it starts
        and ends at the defined location, while maintaining the velocity profile. The
        velocity profile will be limited to max_velocity. Once the max velocity is
        reached it will be maintained until it is time to decelerate to the target
        velocity (wrt the vel_profile). The final velocity profile will go from
        start_velocity to max_velocity, and back down the target_velocity. If we do not
        have enough time to reach max_velocity we will accelerate until it is time to
        decelerate to target_velocity.

        - A start and target orientation can optionally be passed in to generate_path
        - The order of the euler angles is defined by 'axes' on init
        - Quaternion SLERP is used to smoothly transition from start to target
        orientation, following the velocity profile so that we reach the target
        orientation at the same moment we reach out target position and target
        velocity.

        Parameters
        ----------
        pos_profile: position_profiles class
            Must have a step function that takes in a float from 0 to 1, and returns a
            3x1 array. This defines the shape of the desired path, with t(0) defining
            the starting position at [0, 0, 0], and t(1) defining the target at
            [1, 1, 1]. The path planner will do the appropriate warping to the actual
            start and target.
        vel_profile: velocity_profiles class
            Must accept dt on init. Must have a generate function that takes in start
            and target velocities as floats, and returns a 1xN list that transitions
            between them, where N is determined by dt.
        axes: string, Optional (Default: 'rxyz')
            The euler order of state and target orientations.
        verbose: bool, Optional (Default: False)
            True for extra debug prints.
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
            Vector being rotated.
        b: 3xa array of floats
            vector to align to.
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
        Takes a start and target position, along with an optional start and target
        velocity, and generates a trajectory that smoothly accelerates, at a rate
        defined by vel_profile, from start_velocity to max_v, and back to target_v.
        If the path is too short to reach max_v and still decelerate to target_v at a
        rate of max_a, then the path will be slowed to the maximum allowable velocity
        so that we still reach target_velocity at the moment we are at target_position.
        Optionally can pass in a 3D angular state [a, b, g] and target orientation.
        Note that the orientation should be given in euler angles, in the ordered
        specified by axes on init. The orientation path will follow the same velocity
        profile as the position path.

        Parameters
        ----------
        start_position: 3x1 np.array of floats
            The starting position (x, y, z).
        target_position: 3x1 np.array of floats
            The target position (x, y, z).
        max_velocity: float
            The maximum allowable velocity of the path.
        start_velocity: float, Optional (Default: 0)
            The velocity at start of path.
        target_velocity: float, Optional (Default: 0)
            The velocity at end of path.
        start_orientation: 3x1 np.array of floats, Optional (Default: None)
            The orientation at start of path in euler angles, given in the order
            specified on __init__ with the axes parameter (default rxyz). When left as
            `None`, no orientation path will be planned.
        target_orientation: 3x1 np.array of floats, Optional (Default: None)
            The target orientation at the end of the path in euler angles, given in the
            order specified on __init__ with the axes parameter.
        plot: bool, Optional (Default: False)
            Set `True` to plot path profiles for debugging.
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

        # save as self variables so we can check on the next generate call if we
        # need a different profile
        self.start_velocity = start_velocity
        self.target_velocity = target_velocity

        if self.verbose:
            self.log.append(
                f"{c.blue}Generating a path from {start_position} to "
                + f"{target_position}{c.endc}"
            )
            self.log.append(f"{c.blue}max_velocity={self.max_velocity}{c.endc}")
            self.log.append(
                f"{c.blue}start_velocity={self.start_velocity} | "
                + f"target_velocity={self.target_velocity}{c.endc}"
            )

        # calculate the distance between our current state and the target
        target_direction = target_position - start_position
        dist = np.linalg.norm(target_direction)
        target_norm = target_direction / dist

        # the default direction of our path shape
        a = 1 / np.sqrt(3)
        base_norm = np.array([a, a, a])
        # get rotation matrix to rotate our path shape to the target direction
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

        # distance is greater than our ramping up and down distance, add a linear
        # velocity between the ramps to converge to the correct position
        self.remaining_dist = None

        # if the max_v is not attainable in the distance from start to target
        # start dropping it to reach the maximum possible velocity. This is
        # better than scaling the profile as before, because the prior method
        # results in a constant number of steps regarless of distance, which
        # can really slow down short reaches.
        searching_for_valid_velocity_profile = True

        max_v = self.max_velocity
        while searching_for_valid_velocity_profile:
            if max_v <= 0:
                raise ValueError

            if self.starting_dist != 0:
                # Regenerate our velocity curves if start or end v have changed
                self.starting_vel_profile = self.vel_profile.generate(
                    start_velocity=start_velocity, target_velocity=max_v
                )

                # calculate the distance covered ramping from start_velocity to
                # max_v and from max_v to target_velocity
                self.starting_dist = np.sum(self.starting_vel_profile * self.dt)

            if self.ending_dist != 0:
                # if our start and end v are the same, just mirror the curve to
                # avoid regenerating
                if start_velocity == target_velocity:
                    self.ending_vel_profile = self.starting_vel_profile[::-1]
                else:
                    self.ending_vel_profile = self.vel_profile.generate(
                        start_velocity=target_velocity, target_velocity=max_v
                    )[::-1]

                # calculate the distance covered ramping from start_velocity to
                # max_v and from max_v to target_velocity
                self.ending_dist = np.sum(self.ending_vel_profile * self.dt)

            if curve_length > self.starting_dist + self.ending_dist:
                # calculate the remaining steps where we will be at constant max_v
                remaining_dist = curve_length - (self.ending_dist + self.starting_dist)
                constant_speed_steps = int(remaining_dist / max_v / self.dt)

                self.stacked_vel_profile = np.hstack(
                    (
                        self.starting_vel_profile,
                        np.ones(constant_speed_steps) * max_v,
                        self.ending_vel_profile,
                    )
                )
                if plot:
                    self.remaining_dist = remaining_dist
                    self.dist = dist

                searching_for_valid_velocity_profile = False
            elif curve_length == self.starting_dist + self.ending_dist:
                self.stacked_vel_profile = np.hstack(
                    (
                        self.starting_vel_profile,
                        self.ending_vel_profile,
                    )
                )
                if plot:
                    self.remaining_dist = remaining_dist
                    self.dist = dist

                searching_for_valid_velocity_profile = False

            else:
                max_v -= 0.1

        if self.verbose:
            if max_v != self.max_velocity:
                self.log.append(
                    f"{c.yellow}Maximum reachable velocity given acceleration and distance: {max_v:.2f}m/s{c.endc}"
                )
            else:
                self.log.append(
                    f"{c.green}Max velocity reached: {self.max_velocity}m/s{c.endc}"
                )
        self.position_path = []

        # the distance covered over time with respect to our velocity profile
        path_steps = np.cumsum(self.stacked_vel_profile * self.dt)

        # step along our curve, with our next path step being the distance
        # determined by our velocity profile, in the direction of the path curve
        for ii in range(0, len(self.stacked_vel_profile)):
            shiftx = XYZ[0](path_steps[ii])
            shifty = XYZ[1](path_steps[ii])
            shiftz = XYZ[2](path_steps[ii])
            shift = np.array([shiftx, shifty, shiftz])
            self.position_path.append(shift)

        self.position_path = np.asarray(self.position_path)

        # get our 3D vel profile components by differentiating the position path
        # our velocity profile is 1D, to get the 3 velocity components we can
        # just differentiate the position path. Since the distance between steps
        # was determined with our velocity profile, we should still maintain the
        # desired velocities. Note this may break down with complex, high
        # frequency paths
        self.velocity_path = np.asarray(
            np.gradient(self.position_path, self.dt, axis=0)
        )

        # check if we received start and target orientations
        if isinstance(start_orientation, (list, (np.ndarray, np.generic), tuple)):
            if isinstance(target_orientation, (list, (np.ndarray, np.generic), tuple)):
                # Generate the orientation portion of our trajectory.
                # We will use quaternions and SLERP for filtering orientation path
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
                        f"{c.blue}start_orientation={start_orientation} | "
                        + f"target_orientation={target_orientation}{c.endc}"
                    )
            else:
                raise NotImplementedError(
                    f"{c.red}A target orientation is required to generate path{c.endc}"
                )

            self.orientation_path = np.asarray(self.orientation_path)
            # TODO should this be included? look at proper derivation here...
            # https://physics.stackexchange.com/questions/73961/angular-velocity-expressed-via-euler-angles  # pylint: disable=C0301
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
        else:
            self.path = np.hstack((self.position_path, self.velocity_path))

        if plot:
            self._plot(
                start_position=start_position,
                target_position=target_position,
            )

        # Some parameters that are useful to have access to externally,
        # used in nengo-control
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
                f"{c.blue}pos x error: "
                + f"{self.position_path[-1, 0] - target_position[0]}{c.endc}"
            )
            self.log.append(
                f"{c.blue}pos y error: "
                + f"{self.position_path[-1, 1] - target_position[1]}{c.endc}"
            )
            self.log.append(
                f"{c.blue}pos x error: "
                + f"{self.position_path[-1, 2] - target_position[2]}{c.endc}"
            )
            self.log.append(
                f"{c.blue}2norm error at target: "
                + f"{np.linalg.norm(self.position_path[-1] - target_position[:3])}{c.endc}"  # pylint: disable=C0301
            )

            dash = "".join((["-"] * len(max(self.log, key=len))))
            print(f"{c.blue}{dash}{c.endc}")
            for log in self.log:
                print(log)
            print(f"{c.blue}{dash}{c.endc}")

            self.log = []

        err = np.linalg.norm(self.position_path[-1] - target_position)
        if err >= 0.01:
            warnings.warn(
                (
                    f"\n{c.yellow}WARNING: the distance at the end of the "
                    + f"generated path to your desired target position is {err}m."
                    + "If you desire a lower error you can try:"
                    + "\n\t- a path shape with lower frequency terms"
                    + "\n\t- more sample points (set on __init__)"
                    + "\n\t- smaller simulation timestep"
                    + "\n\t- lower maximum velocity and acceleration"
                    + f"\n\t- lower start and end velocities{c.endc}"
                )
            )

        return self.path

    def next(self):
        """
        Returns the next target from the generated path.
        """
        path = self.path[self.n]
        if self.n_timesteps is not None:
            self.n = min(self.n + 1, self.n_timesteps - 1)
        else:
            self.n += 1

        return path

    def next_at_n(self, n):
        """
        Returns the nth point along the path without incrementing any internal coutners.
        if n > len(path) the last point is returned
        """
        if n >= self.n_timesteps:
            n = self.n_timesteps - 1
        path = self.path[n]

        return path

    def convert_to_time(self, path, time_length):
        """Accepts a pregenerated path from current state to target and interpolates
        with respect to the time_limit. The function can then be stepped through to
        reach a target within the specified time.

        Parameters
        ----------
        path: numpy.array
            The output from a subclasses generate_path() function.
        time_length: float
            The desired time to go from state to target [seconds].
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

    def _plot(self, start_position, target_position):
        """
        Only called internally if plot=True in the generate_path call. Plots several
        profiles of the path for debugging. Most of the parameters accessed are saved
        as self variables.

        Parameters
        ----------
        start_position: 3x1 np.array of floats
            The starting (x, y, z) position.
        target_position: 3x1 np.array of floats
            The target (x, y, z) position.
        """
        len_start = len(self.starting_vel_profile)
        len_end = len(self.ending_vel_profile)
        cols = 2 if self.path.shape[1] == 12 else 1

        def _plot3(ax, array):
            ax.plot(array[:, 0], "r")
            ax.plot(array[:, 1], "b")
            ax.plot(array[:, 2], "g")

        def _scatter3(ax, x, y):
            ax.scatter(x, y[0], c="r")
            ax.scatter(x, y[1], c="b")
            ax.scatter(x, y[2], c="g")

        plt.figure(figsize=(8, 8))
        # plot components of the position path, and markers for the start and target
        ax0 = plt.subplot(2, cols, 1)
        ax0.set_title("Position")
        steps = self.position_path.shape[0]
        _plot3(ax0, self.position_path)
        _scatter3(ax0, steps, target_position)
        _scatter3(ax0, 0, start_position)

        # add markers where our velocity ramps stop and end
        # ax0.plot(np.cumsum(self.position_path))
        # ax0.scatter(len_start, self.starting_dist, c="m")
        # if self.remaining_dist is not None:
        #     remain_plus_start = self.remaining_dist + self.starting_dist
        #     ax0.scatter(steps - len_end, remain_plus_start, c="c")
        #     ax0.scatter(steps, self.ending_dist + remain_plus_start, c="k")
        # fill the space signifying the linear portion of the path
        ax0.axvspan(len_start, steps - len_end, alpha=0.25)
        ax0.legend(["x", "y", "z"])

        # plot the components of the velocity profile
        ax1 = plt.subplot(2, cols, 2)
        ax1.set_title("Velocity")
        _plot3(ax1, self.velocity_path)

        # plot the normalized velocities and markers for the limits
        norm = []
        for vel in self.velocity_path:
            norm.append(np.linalg.norm(vel))
        ax1.plot(norm, "y")
        ax1.plot([self.max_velocity] * len(norm), linestyle="--")
        ax1.plot([self.start_velocity] * len(norm), linestyle="-")
        ax1.plot([self.target_velocity] * len(norm), linestyle="--")
        ax1.legend(["dx", "dy", "dz", "norm", "vel limit", "start_vel", "target_vel"])

        # plot the orientation path if it exists
        if self.path.shape[1] == 12:
            ax2 = plt.subplot(2, cols, 3)
            ax2.set_title("Orientation")
            ax2.plot(self.orientation_path)
            ax2.legend(["a", "b", "g"])

            ax3 = plt.subplot(2, cols, 4)
            ax3.set_title("Angular Velocity")
            ax3.plot(self.ang_velocity_path)
            ax3.legend(["da", "db", "dg"])

        plt.tight_layout()

        # plots of given and warped position profile
        curve = []
        x = np.linspace(0, 1, self.n_sample_points)
        for xx in x:
            curve.append(self.pos_profile.step(xx))
        curve = np.array(curve).T

        # plot the shape of the given curve
        plt.figure(figsize=(12, 4))
        ax1 = plt.subplot(131, projection="3d")
        ax1.set_title("Given Curve")
        ax1.plot(curve[0], curve[1], curve[2])

        dist_err = np.linalg.norm(self.position_path[-1] - target_position[:3])

        # plot the transformed curve: scaled, rotated, and shifted
        ax3 = plt.subplot(132, projection="3d")
        ax3.set_title("Warped Curve")
        ax3.plot(*self.warped_xyz.T)
        ax3.scatter(*start_position, label="start")
        ax3.scatter(*target_position, label="target")
        ax3.legend()

        # plot the final path interpolated from the warped curve
        ax3 = plt.subplot(133, projection="3d")
        ax3.set_title("Interpolated Position Path")
        ax3.plot(*self.position_path.T, label=f"error at target={dist_err:.4f}m")
        ax3.scatter(*start_position, label="start")
        ax3.scatter(*target_position, label="target")
        ax3.legend()

        # plot the components of the given curve
        plt.figure(figsize=(8, 8))
        labels = ["X", "Y", "Z"]
        for ii in range(3):
            ax = plt.subplot(3, 3, ii + 1)
            ax.set_title(f"Given {labels[ii]} Shape")
            ax.set_xlabel("Steps [unitless]")
            ax.plot(curve[ii])

        # plot the components of the warped curve
        args = [0, self.n_sample_points - 1]
        plot_args = lambda c, l: {"linestyle": "--", "color": c, "label": l}
        for ii in range(3):
            ax = plt.subplot(3, 3, ii + 4)
            ax.set_title(f"Warped {labels[ii]} Path")
            ax.set_xlabel("Steps [unitless]")
            ax.plot(self.warped_xyz.T[ii])
            ax.hlines(start_position[ii], *args, **plot_args("y", "start"))
            ax.hlines(target_position[ii], *args, **plot_args("g", "target"))
            ax.legend()

        # plot the components of the path from the interpolated function
        err = self.position_path[-1] - target_position
        t = np.arange(0, self.position_path.shape[0]) * self.dt
        args = [0, t[-1]]
        for ii in range(3):
            ax = plt.subplot(3, 3, ii + 7)

            ax.set_title(f"Interpolated {labels[ii]} Path")
            ax.set_xlabel("Time [sec]")
            ax.plot(
                t,
                self.position_path[:, ii],
                label=f"{labels[ii]}_err={err[ii]:.4f}m",
            )
            ax.hlines(start_position[ii], *args, **plot_args("y", "start"))
            ax.hlines(target_position[ii], *args, **plot_args("g", "target"))
            ax.legend()

        plt.tight_layout()
        plt.show()
