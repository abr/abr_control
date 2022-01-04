import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611
from abr_control.controllers.path_planners.orientation import Orientation
from abr_control.controllers.path_planners.path_planner import PathPlanner
from abr_control.utils import transformations as transform

class GaussianPathPlanner(PathPlanner):

    def __init__(self, max_a, max_v, dt, axes='rxyz', verbose=False):
        """
        Parameters
        ----------
        max_a: float
            The acceleration limit of our path
        max_v: float
            The velocity limit of our path
        dt: float
        axes: string, Optional (Default: 'rxyz')
            The euler order of state and target orientations
        """
        self.dt = dt
        self.axes = axes
        self.OrientationPlanner = Orientation(axes=self.axes)
        self.max_v = max_v
        self.max_a = max_a
        self.path = np.zeros((1, 12))
        self.n = 0
        self.n_timesteps = None
        self.target_counter = 0
        self.verbose = verbose

        self.starting_vel_profile = None
        self.ending_vel_profile = None
        self.start_v = 0  # can be overwritten by generate_path
        self.end_v = 0  # can be overwritten by generate_path


    def get_gauss_curve(self, vel):
        """
        generates the left half of the gaussian curve with 3std, with sigma determined
        by the timestep and max_a
        """
        assert vel <= self.max_v

        if vel == self.max_v:
            vel_profile = None
        else:
            # calculate the time needed to reach our maximum velocity from vel
            ramp_up_time = (self.max_v-vel)/self.max_a

            # Amplitude of Gaussian is max speed, get our sigma from here
            s = 1/ ((self.max_v-vel) * np.sqrt(np.pi*2))

            # Go 3 standard deviations so our tail ends get close to zero
            u = 3*s

            # We are generating the left half of the gaussian, so generate our
            # x values from 0 to the mean, with the steps determined by our
            # ramp up time (which itself is determined by max_a)
            x = np.linspace(0, u, int(ramp_up_time/self.dt))

            # Get our gaussian y values, which is our normalized velocity profile
            vel_profile = 1 * (
                    1/(s*np.sqrt(2*np.pi)) * np.exp(-0.5*((x-u)/s)**2)
            )

            # Since the gaussian goes off to +/- infinity, we need to shift our
            # curve down so that it starts at zero
            vel_profile -= vel_profile[0]

            # scale back up so we reach our target v at the end of the curve
            vel_profile *= ((self.max_v-vel) / vel_profile[-1])

            # add to our baseline starting velocity
            vel_profile += vel

        return vel_profile


    def generate_path(
            self, lin_state, lin_target,
            ang_state=None, ang_target=None,
            start_v=None, end_v=None, plot=False):
        """
        Takes a 6D linear state [x, y, z, dx, dy, dz] and target, and generates a trajectory
        between the two within the limits of max_v and max_a. If the 3D components of velocity
        are not known, the summed vector velocity (1D) can be passed in to start_v and end_v.
        If these are used, the values in lin_state and lin_target are overwritten.

        Optionally can pass in a 3D angular state [a, b, g] and target, and generates a 6D
        trajectory [a, b, g, da, db, dg] between the two, with a velocity profile matching the
        linear path.

        Parameters
        ----------
        lin_state: np.array of len 6
            start position and velocity
        target: np.array of size 3
            target position
        start_v: float, Optional (Default: None)
            can set target velocity here in absolute terms if the components are not known
            NOTE: this overwrites the target velocity passed in in dimension 3,4,5 of state
        end_v: float, Optional (Default: None)
            can set target velocity here in absolute terms if the components are not known
            NOTE: this overwrites the target velocity passed in in dimension 3,4,5 of target
        plot:
        """
        if start_v is None:
            start_v=np.linalg.norm(lin_state[3:6])
        if end_v is None:
            end_v=np.linalg.norm(lin_target[3:6])

        # Regenerate our velocity curves if start or end v have changed
        if self.starting_vel_profile is None or self.start_v != start_v:
            self.starting_vel_profile = self.get_gauss_curve(start_v)

        # if our start and end v are the same, just mirror the curve to avoid regenerating
        if start_v == end_v:
            self.ending_vel_profile = self.starting_vel_profile[::-1]

        elif self.ending_vel_profile is None or self.end_v != end_v:
            self.ending_vel_profile = self.get_gauss_curve(end_v)[::-1]

        self.start_v = start_v
        self.end_v = end_v

        if self.verbose:
            print(f"Generating a path from {lin_state} to {lin_target}")
            print(f"max_v={self.max_v} | max_a={self.max_a}")
            print(f"start_v={self.start_v} | end_v={self.end_v}")

        # calculate the distance covered ramping from start_v to max_v
        # and from max_v to end_v
        self.starting_dist = np.sum(self.starting_vel_profile*self.dt)
        self.ending_dist = np.sum(self.ending_vel_profile*self.dt)

        # calculate the distance between our current state and the target
        dist = np.linalg.norm(lin_state[:3] - lin_target[:3])

        # distance is greater than our ramping up and down distance
        # add a linear velocity from between the ramps to converge to the correct position
        if dist >= self.starting_dist + self.ending_dist:
            # calculate the remaining steps where we will be at constant max_v
            remaining_dist = dist - (self.ending_dist + self.starting_dist)
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
            scale = dist / (self.starting_dist + self.ending_dist)
            self.vel_profile = np.hstack((
                scale*self.starting_vel_profile,
                scale*self.ending_vel_profile
            ))

        self.position_path = [lin_state[:3]]
        direction = lin_target[:3] - lin_state[:3]
        direction /= np.linalg.norm(direction)

        # Generate our position path from the velocity curve
        for ii, step in enumerate(self.vel_profile):
            self.position_path.append(self.position_path[-1] + direction*step*self.dt)
        self.position_path = np.asarray(self.position_path)

        # get our 3D vel profile components by differentiating the position path
        self.velocity_path = np.asarray(np.gradient(self.position_path, self.dt, axis=0))

        if isinstance(ang_state, list) or isinstance(ang_state, (np.ndarray, np.generic)):
            # Generate the orientation portion of our trajectory.
            # We will use quaternions and SLERP for filtering from start_quat to target_quat.
            quat0 = transform.quaternion_from_euler(
                ang_state[0],
                ang_state[1],
                ang_state[2],
                axes=self.axes)

            quat1 = transform.quaternion_from_euler(
                ang_target[0],
                ang_target[1],
                ang_target[2],
                axes=self.axes)

            self.orientation_path = self.OrientationPlanner.match_position_path(
                    orientation=quat0,
                    target_orientation=quat1,
                    position_path=self.position_path)

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
                self._plot(lin_state=lin_state, lin_target=lin_target, ang=True)
        else:
            self.angular_planner = False
            self.path = np.hstack((self.position_path, self.velocity_path))
            if plot:
                self._plot(lin_state=lin_state, lin_target=lin_target, ang=False)

        # Some parameters that are useful to have access to externally, used in nengo-control
        self.n_timesteps = len(self.path)
        self.n = 0
        self.time_to_converge = self.n_timesteps * self.dt
        self.target_counter += 1

        if self.verbose:
            print('Time to converge: ', self.time_to_converge)

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


    def _plot(self, lin_state, lin_target, ang=None):
        len_start = len(self.starting_vel_profile)
        len_end = len(self.ending_vel_profile)
        plt.figure()
        if ang:
            cols = 2
        else:
            cols = 1

        plt.subplot(2, cols, 1)
        plt.title('Position')
        steps = self.position_path.shape[0]
        plt.plot(self.position_path[:, 0], 'r')
        plt.plot(self.position_path[:, 1], 'b')
        plt.plot(self.position_path[:, 2], 'g')

        # NOTE debugging start and end position location aligning with path
        if self.verbose:
            print('pos x err: ', self.position_path[-1, 0] - lin_target[0])
            print('pos y err: ', self.position_path[-1, 1] - lin_target[1])
            print('pos x err: ', self.position_path[-1, 2] - lin_target[2])

        plt.scatter(0, lin_state[0], c='r')
        plt.scatter(0, lin_state[1], c='b')
        plt.scatter(0, lin_state[2], c='g')
        plt.scatter(steps, lin_target[0], c='r')
        plt.scatter(steps, lin_target[1], c='b')
        plt.scatter(steps, lin_target[2], c='g')

        # NOTE debugging length of ramp up and ramp down
        # plot distance covered over time
        plt.plot(np.linalg.norm(self.position_path, axis=1))
        plt.scatter(len_start, self.starting_dist, c='m')
        plt.scatter(steps-len_end, self.remaining_dist + self.starting_dist, c='c')
        plt.scatter(steps, self.ending_dist + self.starting_dist + self.remaining_dist, c='k')
        # plot point at the end of the ramp up
        plt.vlines(len_start, 0, np.linalg.norm(self.position_path, axis=1)[len_start])
        # plot point at the start of the ramp up
        plt.vlines(steps-len_end, 0, np.linalg.norm(self.position_path, axis=1)[-len_end])

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
        plt.show()

if __name__ == '__main__':
    path = GaussianPathPlanner(
            max_a=5,
            max_v=5,
            dt=0.001
    )
    path.generate_path(
            lin_state=np.array([0, 0, 0, 3.58, 1.07, 1.43]),
            ang_state=np.array([0, 0, 0]),
            lin_target=np.array([5, 3, 2, 0.87, 0.26, 0.35]),
            ang_target=np.array([0, 0, 3.14]),
            start_v=0,
            end_v=3,
            plot=True
        )
