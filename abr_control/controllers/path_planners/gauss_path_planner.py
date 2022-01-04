import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611
# from abr_control.controllers.path_planners.path_planner import PathPlanner
# from abr_control.controllers import path_planners
from abr_control.utils import transformations as transform
# from gauss_path_planner import GaussPathPlanner

# class NewPathPlanner(path_planners.path_planner.PathPlanner):
class GaussianPathPlanner():

    def __init__(self, max_a, max_v, dt, axes='rxyz'):
            # n_timesteps, dt, startup_steps=None, NED=False, axes='rxyz'):
        """
        NED: boolean, Optional (Default: False)
            whether or not using NED coordinate system where +z is down
        """
        self.dt = dt
        self.axes = axes
        self.max_v = max_v
        self.max_a = max_a

        # calculate the distance we cover ramping up to our target velocity
        self.starting_vel_profile, self.ending_vel_profile = self._get_gauss_profile(
                max_a=self.max_a,
                max_v=self.max_v,
                start_v=0,
                target_v=0
                )

        self.starting_dist = np.sum(self.starting_vel_profile*self.dt)
        self.path = np.zeros((1, 12))
        self.n = 0
        self.n_timesteps = None
        self.target_counter = 0

    #TODO move this into a class with different ramps, or to utils
    def _get_gauss_profile(self, max_a, max_v, start_v, target_v):

        print('start_v: ', start_v)
        # print('path: ', starting_vel_profile[0])
        print('target_v: ', target_v)
        # print('path: ', ending_vel_profile[-1])

        if start_v == max_v:
            starting_profile = None
        else:
            # ramp_up_time = max_v/max_a
            # TODO if we start at a vel > max_v our number of steps becomes negative
            # when getting x and throws an error in the linspace func
            ramp_up_time = (max_v-start_v)/max_a
            # Amplitude of Gaussian is max speed, get our sigma from here
            # s = 1/ (max_v * np.sqrt(np.pi*2))
            s = 1/ ((max_v-start_v) * np.sqrt(np.pi*2))
            # Go 3 standard deviations so our tail ends get close to zero
            u = 3*s
            x = np.linspace(0, u, int(ramp_up_time/self.dt))
            starting_vel_profile = 1 * (
                    1/(s*np.sqrt(2*np.pi)) * np.exp(-0.5*((x-u)/s)**2)
            ) #+ start_v

            # plt.figure()
            # plt.plot(x, starting_vel_profile, label='initial')
            # shift down so our profile starts at zero
            starting_vel_profile -= starting_vel_profile[0]
            # plt.plot(x, starting_vel_profile, label='shift to zero')

            # scale back up so we reach our target v
            starting_vel_profile *= ((max_v-start_v) / starting_vel_profile[-1])
            # plt.plot(x, starting_vel_profile, label='scale_to_max_v')

            # add to our baseline starting velocity
            starting_vel_profile += start_v
            # plt.plot(x, starting_vel_profile, label='add to baseline')
            # plt.legend()
            # plt.show()

        # if we start and end at the same velocity, mirror the vel curve
        if start_v == target_v:
                ending_vel_profile = starting_vel_profile[::-1]
        else:
            if target_v == max_v:
                ending_vel_profile = None
            else:
                ramp_down_time = (max_v-target_v)/max_a
                # Amplitude of Gaussian is max speed, get our sigma from here
                s = 1/ ((max_v-target_v) * np.sqrt(np.pi*2))
                # Go 3 standard deviations so our tail ends get close to zero
                u = 3*s
                x = np.linspace(0, u, int(ramp_down_time/self.dt))
                ending_vel_profile = 1 * (
                        1/(s*np.sqrt(2*np.pi)) * np.exp(-0.5*((x-u)/s)**2)
                ) #+ target_v

                # shift down so our profile starts at zero
                ending_vel_profile -= ending_vel_profile[0]

                # scale back up so we reach our target v
                ending_vel_profile *= ((max_v-target_v) / ending_vel_profile[-1])

                # add to our baseline ending velocity
                ending_vel_profile += target_v

                ending_vel_profile = ending_vel_profile[::-1]

        # ending_vel_profile = np.insert(ending_vel_profile, -1, target_v)
        # print('length start: ', len(starting_vel_profile))
        # print('length end: ', len(ending_vel_profile))

        return starting_vel_profile, ending_vel_profile

    def _get_linear_profile(self, max_a, max_v, start_v, target_v):
        # LINEAR RAMP FOR DEBUGGING
        starting_vel_profile = np.linspace(start_v, max_v, int(((max_v-start_v)/max_a)/self.dt))
        ending_vel_profile = np.linspace(max_v,  target_v, int(((max_v-target_v)/max_a)/self.dt))

        print('length start: ', len(starting_vel_profile))
        print('length end: ', len(ending_vel_profile))
        return starting_vel_profile, ending_vel_profile


    def generate_path(self, state, target, max_v=None, max_a=None, start_v=None, target_v=None, plot=False):
        """
        Creates a circular path using a gaussian to get to max velocity for the first
        loop and a constant velocity, then ends in the same gaussian curve to slow down

        NOTE that the path starts along the +x axis, but can be shifted by the axis_offset using the right hand rule

        Parameters
        ----------
        state: np.array of size 3
            start position, used to set z and xy offset [m]
        circle_origin: np.array of size 3
            x and y set the center of the circle
            if circle_origin z does not equal the state z, then
            a helical path will be taken, circling the circle_origin xy,
            from the state z to the target z, with a circle of radius r
            Note that a path from state to the start of the circle is not set
        r: float, Optional, (Default: None)
            radius of circle [m]
            if None the radius is set by the distance from state to circle origin
            If radius is set to be different, than a gaussian path planner will be
            used to get to the start of the circle
        loops: float, Optional (Default: 1)
            number of loop arounds circle
        direction: string either 'cw' or 'ccw'
            sets the direction of movement
            This does not follow the right hand rule when in NED, but the reverse since it is
            more intuitive to think of cw and ccw from the perspective of looking down
        axis_offset: float, Optional (Default: 0)
            the path starts along the +x axis, this value in radians will offset the starting
            point of the circle path using the right hand rule
        start_v: float, Optional (Default: None)
            can set target velocity here in absolute terms if the components are not known
            NOTE: this overwrites the target velocity passed in in dimension 3,4,5 of state
        target_v: float, Optional (Default: None)
            can set target velocity here in absolute terms if the components are not known
            NOTE: this overwrites the target velocity passed in in dimension 3,4,5 of target
        """

        # if we have non-zero start or target_vel, get our vel profile
        if abs(sum(state[3:6]) + sum(target[3:6])) > 0 or start_v is not None or target_v is not None:
            if max_v is None:
                max_v = self.max_v
            if max_a is None:
                max_a = self.max_a

            if start_v is None:
                start_v=np.linalg.norm(state[3:6])
            if target_v is None:
                target_v=np.linalg.norm(target[3:6])

            print('Generating path with %.3f start and %.3f target vel'
                    % (start_v, target_v))

            starting_vel_profile, ending_vel_profile = self._get_gauss_profile(
                    max_a=max_a,
                    max_v=max_v,
                    start_v=start_v,
                    target_v=target_v,
                    )

            # starting_vel_profile, ending_vel_profile = self._get_linear_profile(
            #         max_a=max_a,
            #         max_v=max_v,
            #         start_v=start_v,
            #         target_v=target_v,
            #         )

            # starting_dist = np.sum(starting_vel_profile*self.dt)
            starting_dist = np.sum(starting_vel_profile*self.dt)
            ending_dist = np.sum(ending_vel_profile*self.dt)
            #TODO delete these two
            self.starting_dist = starting_dist
            self.ending_dist = ending_dist

        else:
            print('Generating path with zero start and target vel')
            max_v = self.max_v
            starting_vel_profile = self.starting_vel_profile
            ending_vel_profile = self.ending_vel_profile
            starting_dist = self.starting_dist
            # same startup and slowdown
            ending_dist = self.starting_dist

        state = np.asarray(state)
        self.state = state

        dist = np.linalg.norm(state[:3] - target[:3])

        if dist >= starting_dist + ending_dist:
            # we use the same profile to slow down at the end, so determine the number of
            # steps at const speed required
            remaining_dist = dist - (ending_dist + starting_dist)
            constant_speed_steps = int(remaining_dist/ max_v / self.dt)
            # print('const steps: ', constant_speed_steps)
            # print('rem dist: ', remaining_dist)
            # print('tot dist: ', dist)
            # print('end dist: ', ending_dist)
            # print('start dist: ', starting_dist)
            vel_profile = np.hstack((
                starting_vel_profile,
                np.ones(constant_speed_steps) * max_v,
                ending_vel_profile
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
            scale = dist / (starting_dist + ending_dist)
            vel_profile = np.hstack((
                scale*starting_vel_profile,
                scale*ending_vel_profile
            ))

        self.position_path = [state[:3]]
        direction = target[:3] - state[:3]
        direction /= np.linalg.norm(direction)

        for ii, step in enumerate(vel_profile):
            self.position_path.append(self.position_path[-1] + direction*step*self.dt)
        self.position_path = np.asarray(self.position_path)

        # Alternatively we can do the same thing we did for position, except we remove the dt scaling
        self.velocity_path = np.asarray(np.gradient(self.position_path, self.dt, axis=0))
        # self.velocity_path = vel_profile[:, None] * direction[:, None].T

        # get our distance to target to get a normalized profile to use for orientation
        # can probably used normalized position path here
        error = []
        for ee in self.position_path:
            error.append(np.sqrt(np.sum((self.position_path[-1] - ee) ** 2)))
        error /= dist
        error = 1 - error

        quat0 = transform.quaternion_from_euler(
            state[6],
            state[7],
            state[8],
            axes=self.axes)

        quat1 = transform.quaternion_from_euler(
            target[6],
            target[7],
            target[8],
            axes=self.axes)

        self.orientation_path = []

        for step in error:
            quat = transform.quaternion_slerp(
                quat0=quat0,
                quat1=quat1,
                fraction=step
            )
            self.orientation_path.append(
                    transform.euler_from_quaternion(
                        quat,
                        axes=self.axes
                    )
            )

        self.orientation_path = np.asarray(self.orientation_path)
        self.ang_velocity_path = np.asarray(
                np.gradient(self.orientation_path, self.dt, axis=0))

        # plan our orientation path

        #TODO delete these two lines
        self.len_start = len(starting_vel_profile)
        self.len_end = len(ending_vel_profile)
        if plot:
            self._plot(state=state, target=target)

        self.ang_velocity_path *= 0
        self.path = np.hstack(
                    (np.hstack(
                        (np.hstack((self.position_path, self.velocity_path)),
                            self.orientation_path)),
                        self.ang_velocity_path)
                )

        self.n_timesteps = len(self.path)
        self.n = 0
        self.time_to_converge = self.n_timesteps * self.dt
        print('Time to converge: ', self.time_to_converge)
        self.target_counter += 1

        # save as a self variable so we don't zero out target vel on our last step if we want non-zero vel
        self.target_v = target_v


        return self.position_path, self.velocity_path, self.orientation_path #, self.ang_velocity_path

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
        # some path planner may not end with zero target velocity depending on
        # their parameters this will assure that you have zero target velocity
        # when the filter is positionally at the final target
        if self.n_timesteps is not None:
            if self.n == self.n_timesteps - 1:
                # if we have non-zero target_v, do not zero out
                if self.target_v == 0:
                    path[3:6] = np.zeros(3)
                    path[9:] = np.zeros(3)

        return path


    def _plot(self, state, target):
        plt.figure()

        plt.subplot(221)
        plt.title('Position')
        steps = self.position_path.shape[0]
        plt.plot(self.position_path[:, 0], 'r')
        plt.plot(self.position_path[:, 1], 'b')
        plt.plot(self.position_path[:, 2], 'g')

        # NOTE debugging start and end position location aligning with path
        print('pos x err: ', self.position_path[-1, 0] - target[0])
        print('pos y err: ', self.position_path[-1, 1] - target[1])
        print('pos x err: ', self.position_path[-1, 2] - target[2])

        plt.scatter(0, state[0], c='r')
        plt.scatter(0, state[1], c='b')
        plt.scatter(0, state[2], c='g')
        plt.scatter(steps, target[0], c='r')
        plt.scatter(steps, target[1], c='b')
        plt.scatter(steps, target[2], c='g')

        # NOTE debugging length of ramp up and ramp down
        # plot distance covered over time
        plt.plot(np.linalg.norm(self.position_path, axis=1))
        plt.scatter(self.len_start, self.starting_dist, c='m')
        plt.scatter(steps-self.len_end, self.remaining_dist + self.starting_dist, c='c')
        plt.scatter(steps, self.ending_dist + self.starting_dist + self.remaining_dist, c='k')
        # plot point at the end of the ramp up
        plt.vlines(self.len_start, 0, np.linalg.norm(self.position_path, axis=1)[self.len_start])
        # plot point at the start of the ramp up
        plt.vlines(steps-self.len_end, 0, np.linalg.norm(self.position_path, axis=1)[-self.len_end])

        plt.legend(['x', 'y', 'z'])

        plt.subplot(223)
        plt.title('Velocity')
        plt.plot(self.velocity_path[:, 0], 'r')
        plt.plot(self.velocity_path[:, 1], 'b')
        plt.plot(self.velocity_path[:, 2], 'g')

        # NOTE for debugging components of vel start and target alignment with path (if components known)
        # plt.scatter(0, state[3], c='r')
        # plt.scatter(0, state[4], c='b')
        # plt.scatter(0, state[5], c='g')
        # plt.scatter(steps, target[3], c='r')
        # plt.scatter(steps, target[4], c='b')
        # plt.scatter(steps, target[5], c='g')
 
        norm = []
        for vel in self.velocity_path:
            norm.append(np.linalg.norm(vel))
        plt.plot(norm, 'y')
        plt.legend(['dx', 'dy', 'dz', 'norm'])
        # ax = plt.subplot(413)
        # ax.set_aspect('equal')
        # plt.scatter(self.position_path.T[0], self.position_path.T[1])

        plt.subplot(222)
        plt.title('Orientation')
        plt.plot(self.orientation_path)
        plt.legend(['a', 'b', 'g'])

        plt.subplot(224)
        plt.title('Angular Velocity')
        plt.plot(self.ang_velocity_path)
        plt.legend(['da', 'db', 'dg'])

        plt.show()

if __name__ == '__main__':
    dt = 0.005
    path = GaussianPathPlanner(
            max_a=5,
            max_v=5,
            dt=0.001
    )
    path.generate_path(
            state=np.array([
                0, 0, 0,
                # 0, 0, 0,
                3.58764495, 1.07629349, 1.43505798,
                0, 0, 0,
                0, 0, 0
            ]),
            target=np.array([
                5, 3, 2,
                # 1, 2.8733725, 2,
                # 0, 0, 0,
                0.87978059, 0.26393418, 0.35191224,
                0, 0, 3.14,
                0, 0, 0
            ]),
            # start_v=1,
            # target_v=3,
            start_v=0,
            target_v=3,
            plot=True
        )
