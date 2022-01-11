import numpy as np
import warnings
import scipy.interpolate
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611
from abr_control.controllers.path_planners.orientation import Orientation
from abr_control.utils import transformations as transform

class PathPlanner():

    def __init__(self, max_v, dt, pos_profile, vel_profile, axes='rxyz', n_sample_points=1000, verbose=False):
        """
        Takes the desired shape of path as either a list of points or a function
        The shape must be defined in the domain of t=[0,1] if a function, and
        has to be defined with its start at [0,0,0] and end at [1,1,1].

        The path planner will return a path in the desired shape, but stretched
        from the start to the target position. The path will then be sampled
        over time to output a trajectory that is limited by the user defined
        maximum velocity, maximum acceleration, and start and target velocities

        Parameters
        ----------
        max_v: float
            The velocity limit of our path
        dt: float
        shape: function or 3d array
            array of 3d points that define the shape of the desired path
            OR function that outputs a 3D position given a time
            NOTE: the shape should be defined from the origin to [1,1,1],
            where [0,0,0] is the start of the path, and [1,1,1] is the target.
            If using the function, these should be the values at t=0 and t=1.

            The path planner will do the stretch and rotation to align the shape
            with the given start and target
        axes: string, Optional (Default: 'rxyz')
            The euler order of state and target orientations
        n_sample_points: Optional (Default: 1000):
            the number of points used to interpolate our curve when calculating
            distances. If your shape has higher frequency terms, increase this
            to improve how closely the curve matches.
        """
        self.n_sample_points = n_sample_points
        self.pos_profile = pos_profile
        self.vel_profile = vel_profile
        self.dt = dt
        self.axes = axes
        self.OrientationPlanner = Orientation(axes=self.axes)
        self.max_v = max_v
        self.path = np.zeros((1, 12))
        self.n = 0
        self.n_timesteps = None
        self.target_counter = 0
        self.verbose = verbose

        self.starting_vel_profile = None
        self.ending_vel_profile = None
        self.start_v = 0  # can be overwritten by generate_path
        self.end_v = 0  # can be overwritten by generate_path


    def align_vectors(self, a, b):
        """
        Takes vectors a and b and returns rotation matrix to align a to b
        """
        b = b / np.linalg.norm(b) # normalize a
        a = a / np.linalg.norm(a) # normalize b
        v = np.cross(a, b)
        c = np.dot(a, b)

        v1, v2, v3 = v
        h = 1 / (1 + c)

        Vmat = np.array([[0, -v3, v2],
                    [v3, 0, -v1],
                    [-v2, v1, 0]])

        R = np.eye(3, dtype=np.float64) + Vmat + (Vmat.dot(Vmat) * h)
        return R


    def generate_path(
            self, position, target_position,
            start_v=0, end_v=0,
            orientation=None, target_orientation=None,
            plot=False):
        """
        Takes a start and target position, along with an optional start and target velocity,
        and generates a trajectory that smoothly accelerates, at a rate defined by max_a,
        from start_v to max_v, and back to target_v. If the path is too short to reach max_v
        and still decelerate to target_v at a rate of max_a, then the path will be slowed to
        the maximum allowable velocitys so that we still reach end_v at the moment we are at
        target_position.

        Optionally can pass in a 3D angular state [a, b, g] and target orientation. Note that
        the orientation should be given in euler angles, in the ordered specified by axes on init.
        The orientation path will follow the same velocity profile as the position path. If no
        target orientation is given, the orientation path will be along the path of motion.

        Parameters
        ----------
        position: np.array of len 3
            start position
        target: np.array of size 3
            target position
        start_v: float, Optional (Default: 0)
            velocity at start of path
        end_v: float, Optional (Default: 0)
            velocity at end of path
        orientation: np.array of len 3, Optional (Default: None)
            orientation at start of path in euler angles, given in the order specified
            on __init__ with the axes parameter (default rxyz).
            When left as None no orientation path will be planned
        target_orientation: np.array of len 3, Optional (Default: None)
            the target orientation at the end of the path in euler angles, given in the
            order specified on __init__ with the axes parameter (default rxzyz)

            When left as None when a start orientation is provided, the orientation path
            will point along the direction of movement
        plot:
        """
        # Regenerate our velocity curves if start or end v have changed
        if self.starting_vel_profile is None or self.start_v != start_v:
            self.starting_vel_profile = self.vel_profile.generate(start=start_v, target=self.max_v)

        # if our start and end v are the same, just mirror the curve to avoid regenerating
        if start_v == end_v:
            self.ending_vel_profile = self.starting_vel_profile[::-1]

        elif self.ending_vel_profile is None or self.end_v != end_v:
            self.ending_vel_profile = self.vel_profile.generate(start=end_v, target=self.max_v)[::-1]

        self.start_v = start_v
        self.end_v = end_v

        if self.verbose:
            print(f"Generating a path from {position} to {target_position}")
            print(f"max_v={self.max_v}")
            print(f"start_v={self.start_v} | end_v={self.end_v}")

        # calculate the distance covered ramping from start_v to max_v
        # and from max_v to end_v
        self.starting_dist = np.sum(self.starting_vel_profile*self.dt)
        self.ending_dist = np.sum(self.ending_vel_profile*self.dt)

        # calculate the distance between our current state and the target
        target_direction = target_position - position
        dist = np.linalg.norm(target_direction)
        target_norm = target_direction/dist

        # the default direction of our path shape
        a = 1/np.sqrt(3)
        base_norm = np.array([a, a, a])
        # get the rotation matrix to rotate our path shape to the target direction
        R = self.align_vectors(base_norm, target_norm)

        # get the length travelled along our stretched curve
        curve_dist_steps = []
        warped_xyz = []
        for ii, t in enumerate(np.linspace(0, 1, self.n_sample_points)):
            # stretch our curve shape to be the same length as target-start
            # rotate our curve to align with the direction target-start
            # shift our curve to begin at the start state
            warped_target = np.dot(R, (1/np.sqrt(3))*self.pos_profile.step(t)*dist) + position
            warped_xyz.append(warped_target)
            if t>0:
                curve_dist_steps.append(np.linalg.norm(warped_xyz[ii]-warped_xyz[ii-1]))
            else:
                curve_dist_steps.append(0)

        # get the cumulative distance covered
        dist_steps = np.cumsum(curve_dist_steps)
        curve_length = np.sum(curve_dist_steps)
        # create functions that return our path at a given distance
        # along that curve
        self.warped_xyz = np.array(warped_xyz)
        X = scipy.interpolate.interp1d(dist_steps, self.warped_xyz.T[0])
        Y = scipy.interpolate.interp1d(dist_steps, self.warped_xyz.T[1])
        Z = scipy.interpolate.interp1d(dist_steps, self.warped_xyz.T[2])
        XYZ = [X, Y, Z]

        # distance is greater than our ramping up and down distance
        # add a linear velocity from between the ramps to converge to the correct position
        if curve_length >= self.starting_dist + self.ending_dist:
            # calculate the remaining steps where we will be at constant max_v
            remaining_dist = curve_length - (self.ending_dist + self.starting_dist)
            constant_speed_steps = int(remaining_dist/ self.max_v / self.dt)

            self.vel_profile = np.hstack((
                self.starting_vel_profile,
                np.ones(constant_speed_steps) * self.max_v,
                self.ending_vel_profile
            ))
            # TODO delete these two lines
            self.remaining_dist = remaining_dist
            self.dist = dist
        else:
            # scale our profile
            # TODO to do this properly we should evaluate the integral to get the t where
            # the sum of the profile is half our travel distance. This way we maintain
            # the same acceleration profile instead of maintaining the same number of steps
            # and accelerating more slowly
            # NOTE ERROR: if we have non-zero start or end velocities this scales us away from
            # velocity
            scale = curve_length / (self.starting_dist + self.ending_dist)
            self.vel_profile = np.hstack((
                scale*self.starting_vel_profile,
                scale*self.ending_vel_profile
            ))


        self.position_path = [position]

        # the distance along our curve we should be over time
        # to maintain the desired velocity limits
        path_steps = np.cumsum(self.vel_profile*self.dt)
        # step along our curve, with our next path step being the
        # distance determined by our velocity limits, in the direction
        # of the path curve
        for ii in range(1, len(self.vel_profile)):
            # normalize step for the normalized shape functions
            tt = path_steps[ii]/curve_length

            shiftx = XYZ[0](path_steps[ii])
            shifty = XYZ[1](path_steps[ii])
            shiftz = XYZ[2](path_steps[ii])
            shift = np.array([shiftx, shifty, shiftz])
            self.position_path.append(shift)

        self.position_path = np.asarray(self.position_path)

        # get our 3D vel profile components by differentiating the position path
        self.velocity_path = np.asarray(np.gradient(self.position_path, self.dt, axis=0))

        if isinstance(orientation, list) or isinstance(orientation, (np.ndarray, np.generic)):
            if isinstance(target_orientation, list) or isinstance(target_orientation, (np.ndarray, np.generic)):
                # Generate the orientation portion of our trajectory.
                # We will use quaternions and SLERP for filtering from start_quat to target_quat.
                quat0 = transform.quaternion_from_euler(
                    orientation[0],
                    orientation[1],
                    orientation[2],
                    axes=self.axes)

                quat1 = transform.quaternion_from_euler(
                    target_orientation[0],
                    target_orientation[1],
                    target_orientation[2],
                    axes=self.axes)

                self.orientation_path = self.OrientationPlanner.match_position_path(
                        orientation=quat0,
                        target_orientation=quat1,
                        position_path=self.position_path)
            else:
                raise NotImplementedError ("A target orientation is required to generate a path")

            self.orientation_path = np.asarray(self.orientation_path)
            # TODO should this be included? look at proper derivation here...
            # https://physics.stackexchange.com/questions/73961/angular-velocity-expressed-via-euler-angles
            self.ang_velocity_path = np.asarray(
                    np.gradient(self.orientation_path, self.dt, axis=0))

            self.path = np.hstack(
                        (np.hstack(
                            (np.hstack((self.position_path, self.velocity_path)),
                                self.orientation_path)),
                            self.ang_velocity_path)
                    )
            self.angular_planner = True
            if plot:
                self._plot(position=position, target_position=target_position, ang=True)
        else:
            self.angular_planner = False
            self.path = np.hstack((self.position_path, self.velocity_path))
            if plot:
                self._plot(position=position, target_position=target_position, ang=False)

        # Some parameters that are useful to have access to externally, used in nengo-control
        self.n_timesteps = len(self.path)
        self.n = 0
        self.time_to_converge = self.n_timesteps * self.dt
        self.target_counter += 1

        if self.verbose:
            print('Time to converge: ', self.time_to_converge)
            print('dt: ', self.dt)
            print('pos x error: ', self.position_path[-1, 0] - target_position[0])
            print('pos y error: ', self.position_path[-1, 1] - target_position[1])
            print('pos x error: ', self.position_path[-1, 2] - target_position[2])
            print('2norm error at target: ', np.linalg.norm(self.position_path[-1] - target_position[:3]))

        err = np.linalg.norm(self.position_path[-1] - target_position)
        if err >= 0.01:
            yellow = '\u001b[33m'
            endc = '\033[0m'
            warnings.warn((
                f"\n{yellow}WARNING: the distance at the end of the generated path to your " +
                f"desired target position is {err}m. If you desire a lower error you can try:" +
                f"\n\t- a path shape with lower frequency terms" +
                f"\n\t- more sample points (set on __init__)" +
                f"\n\t- smaller simulation timestep" +
                f"\n\t- lower maximum velocity and acceleration" +
                f"\n\t- lower start and end velocities{endc}"))


        return self.path

    def next(self):
        """ Returns the next target from the generated path
        """
        # position = self.position_path[self.n]  # pylint: disable=E0203
        # velocity = self.velocity_path[self.n]  # pylint: disable=E0203
        # orientation = self.orientation_path[self.n] # pylint: disable=E0203
        path = self.path[self.n]
        if self.n_timesteps is not None:
            self.n = min(self.n + 1, self.n_timesteps - 1)
        else:
            self.n += 1

        return path


    def _plot(self, position, target_position, ang=None):
        len_start = len(self.starting_vel_profile)
        len_end = len(self.ending_vel_profile)
        plt.figure()
        if ang:
            cols = 2
        else:
            cols = 1

        ax_fillbetween = plt.subplot(2, cols, 1)
        plt.title('Position')
        steps = self.position_path.shape[0]
        plt.plot(self.position_path[:, 0], 'r')
        plt.plot(self.position_path[:, 1], 'b')
        plt.plot(self.position_path[:, 2], 'g')

        plt.scatter(0, position[0], c='r')
        plt.scatter(0, position[1], c='b')
        plt.scatter(0, position[2], c='g')
        plt.scatter(steps, target_position[0], c='r')
        plt.scatter(steps, target_position[1], c='b')
        plt.scatter(steps, target_position[2], c='g')

        # NOTE debugging length of ramp up and ramp down
        # plot distance covered over time
        plt.plot(np.linalg.norm(self.position_path, axis=1))
        plt.scatter(len_start, self.starting_dist, c='m')
        plt.scatter(steps-len_end, self.remaining_dist + self.starting_dist, c='c')
        plt.scatter(steps, self.ending_dist + self.starting_dist + self.remaining_dist, c='k')
        # plot point at the end of the ramp up
        # plt.vlines(len_start, 0, np.linalg.norm(self.position_path, axis=1)[len_start])
        # plot point at the start of the ramp up
        # plt.vlines(steps-len_end, 0, np.linalg.norm(self.position_path, axis=1)[-len_end])
        ax_fillbetween.axvspan(len_start, steps-len_end, alpha=0.25, label='linear velocity')

        plt.legend(['x', 'y', 'z'])

        plt.subplot(2, cols, 2)
        plt.title('Velocity')
        plt.plot(self.velocity_path[:, 0], 'r')
        plt.plot(self.velocity_path[:, 1], 'b')
        plt.plot(self.velocity_path[:, 2], 'g')

        norm = []
        for vel in self.velocity_path:
            norm.append(np.linalg.norm(vel))
        plt.plot(norm, 'y')
        plt.plot([self.max_v]*len(norm), linestyle='--')
        plt.plot([self.start_v]*len(norm), linestyle='--')
        plt.plot([self.end_v]*len(norm), linestyle='--')
        plt.legend(['dx', 'dy', 'dz', 'norm', 'vel limit', 'start_v', 'end_v'])

        if ang:
            plt.subplot(2, cols, 3)
            plt.title('Orientation')
            plt.plot(self.orientation_path)
            plt.legend(['a', 'b', 'g'])

            plt.subplot(2, cols, 4)
            plt.title('Angular Velocity')
            plt.plot(self.ang_velocity_path)
            plt.legend(['da', 'db', 'dg'])

        plt.tight_layout()

        # get the shape of the given curve
        curve = []
        x = np.linspace(0, 1, self.n_sample_points)
        for xx in x:
            curve.append(self.pos_profile.step(xx))
        curve = np.array(curve).T

        # plot the shape of the given curve
        plt.figure()
        ax1 = plt.subplot(131, projection='3d')
        ax1.set_title('Given Curve')
        ax1.plot(curve[0], curve[1], curve[2])

        xerr = self.position_path[-1, 0] - target_position[0]
        yerr = self.position_path[-1, 1] - target_position[1]
        zerr = self.position_path[-1, 2] - target_position[2]
        dist_err = np.linalg.norm(self.position_path[-1] - target_position[:3])

        # plot the transformed curve
        # this is scaled, rotated, and shifted
        ax3 = plt.subplot(132, projection='3d')
        ax3.set_title('Shifted Curve')
        ax3.plot(self.warped_xyz.T[0], self.warped_xyz.T[1], self.warped_xyz.T[2])
        ax3.scatter(position[0], position[1], position[2], label='start')
        ax3.scatter(target_position[0], target_position[1], target_position[2], label='target')
        ax3.legend()

        # plot the final path interpolated from the warped curve
        ax3 = plt.subplot(133, projection='3d')
        ax3.set_title('Interpolated Position Path')
        ax3.plot(self.position_path[:, 0], self.position_path[:, 1], self.position_path[:, 2], label=f"error at target={dist_err:.4f}m")
        ax3.scatter(position[0], position[1], position[2], label='start')
        ax3.scatter(target_position[0], target_position[1], target_position[2], label='target')
        ax3.legend()


        # plot the components of the given curve
        plt.figure()
        plt.subplot(3,3,1)
        plt.title('Given X Shape')
        plt.xlabel('Steps [unitless]')
        plt.plot(curve[0])
        plt.subplot(3,3,2)
        plt.title('Given Y Shape')
        plt.xlabel('Steps [unitless]')
        plt.plot(curve[1])
        plt.subplot(3,3,3)
        plt.title('Given Z Shape')
        plt.xlabel('Steps [unitless]')
        plt.plot(curve[2])

        # plot the components of the warped curve
        plt.subplot(3,3,4)
        plt.title('Warped X Path')
        plt.xlabel('Steps [unitless]')
        plt.plot(self.warped_xyz.T[0])
        plt.hlines(position[0], 0, self.n_sample_points, linestyle='--', color='y', label='start')
        plt.hlines(target_position[0], 0, self.n_sample_points, linestyle='--', color='g', label='target')
        plt.legend()
        plt.subplot(3,3,5)
        plt.title('Warped Y Path')
        plt.xlabel('Steps [unitless]')
        plt.plot(self.warped_xyz.T[1])
        plt.hlines(position[1], 0, self.n_sample_points, linestyle='--', color='y', label='start')
        plt.hlines(target_position[1], 0, self.n_sample_points, linestyle='--', color='g', label='target')
        plt.legend()
        plt.subplot(3,3,6)
        plt.title('Warped Z Path')
        plt.xlabel('Steps [unitless]')
        plt.plot(self.warped_xyz.T[2])
        plt.hlines(position[2], 0, self.n_sample_points, linestyle='--', color='y', label='start')
        plt.hlines(target_position[2], 0, self.n_sample_points, linestyle='--', color='g', label='target')
        plt.legend()

        # plot the components of the path from the interpolated function
        t = np.arange(0, self.position_path.shape[0])
        t = self.dt * np.asarray(t)

        plt.subplot(3,3,7)
        plt.title('Interpolated X Path')
        plt.plot(t, self.position_path[:, 0], label=f"x_err={xerr:.4f}m")
        plt.hlines(position[0], 0, t[-1], linestyle='--', color='y', label='start')
        plt.hlines(target_position[0], 0, t[-1], linestyle='--', color='g', label='target')
        plt.xlabel('Time [sec]')
        plt.legend()
        plt.subplot(3,3,8)
        plt.title('Interpolated Y Path')
        plt.plot(t, self.position_path[:, 1], label=f"y_err={yerr:.4f}m")
        plt.hlines(position[1], 0, t[-1], linestyle='--', color='y', label='start')
        plt.hlines(target_position[1], 0, t[-1], linestyle='--', color='g', label='target')
        plt.xlabel('Time [sec]')
        plt.legend()
        plt.subplot(3,3,9)
        plt.title('Interpolated Z Path')
        plt.plot(t, self.position_path[:, 2], label=f"z_err={zerr:.4f}m")
        plt.hlines(position[2], 0, t[-1], linestyle='--', color='y', label='start')
        plt.hlines(target_position[2], 0, t[-1], linestyle='--', color='g', label='target')
        plt.xlabel('Time [sec]')
        plt.legend()
        plt.tight_layout()

        plt.show()
