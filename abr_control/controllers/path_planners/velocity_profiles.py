"""
Functions that return a 1D array of velocities from a desired start velocity to
a target velocity.
"""
import numpy as np


class VelProf:
    def __init__(self, dt):
        """
        Must take the sim timestep on init, as the path_planner requires it.
        """
        self.dt = dt

    def generate(self, start_velocity, target_velocity):
        """
        Takes start and target velocities as a float, and returns a 1xN array
        of velocities that go from start to target.
        """
        raise NotImplementedError


class Gaussian(VelProf):
    def __init__(self, dt, acceleration, n_sigma=3):
        """
        Velocity profile that follows a gaussian curve.

        Parameters
        ----------
        dt: float
            The timestep (in seconds).
        acceleration: float
            The acceleration that defines our velocity curve.
        n_sigma: int, Optional (Default: 3)
            How many standard deviations of the gaussian function to use for the
            velocity profile. The default value of 3 gives a smooth acceleration and
            deceleration. A slower ramp up can be achieved with a larger sigma, and a
            faster ramp up by decreasing sigma. Note that the curve gets shifted so
            that it starts at zero, since the gaussian has a domain of [-inf, inf].
            At sigma==3 we get close to zero and have a smooth ramp up.
        """
        self.acceleration = acceleration
        self.n_sigma = n_sigma

        super().__init__(dt=dt)

    def generate(self, start_velocity, target_velocity):
        """
        Generates the left half of the gaussian curve with n_sigma std, with
        sigma determined by the timestep and max_a.

        Parameters
        ----------
        start_velocity: float
            The starting velocity in the curve.
        target_velocity: float
            The ending velocity in the curve.
        """
        # calculate the time needed to reach our maximum velocity from vel
        ramp_up_time = (target_velocity - start_velocity) / self.acceleration

        # Amplitude of Gaussian is max speed, get our sigma from here
        s = 1 / ((target_velocity - start_velocity) * np.sqrt(np.pi * 2))

        # Go 3 standard deviations so our tail ends get close to zero
        u = self.n_sigma * s

        # We are generating the left half of the gaussian, so generate our
        # x values from 0 to the mean, with the steps determined by our
        # ramp up time (which itself is determined by max_a)
        x = np.linspace(0, u, int(ramp_up_time / self.dt))

        # Get our gaussian y values, which is our normalized velocity profile
        vel_profile = 1 * (
            1 / (s * np.sqrt(2 * np.pi)) * np.exp(-0.5 * ((x - u) / s) ** 2)
        )

        # Since the gaussian goes off to +/- infinity, we need to shift our
        # curve down so that it start_velocitys at zero
        vel_profile -= vel_profile[0]

        # scale back up so we reach our target v at the end of the curve
        vel_profile *= (target_velocity - start_velocity) / vel_profile[-1]

        # add to our baseline starting velocity
        vel_profile += start_velocity

        return vel_profile


class Linear(VelProf):
    def __init__(self, dt, acceleration):
        """
        Velocity profile that follows a linear curve.

        Parameters
        ----------
        dt: float
            The timestep (in seconds).
        acceleration: float
            The acceleration that defines the velocity curve.
        """
        self.acceleration = acceleration

        super().__init__(dt=dt)

    def generate(self, start_velocity, target_velocity):
        """
        Generates a linear ramp from start to target velocity, with a slope of
        self.acceleration.

        Parameters
        ----------
        start_velocity: float
            The starting velocity in the curve.
        target_velocity: float
            The ending velocity in the curve.
        """

        vdiff = target_velocity - start_velocity
        t = vdiff / self.acceleration
        steps = t / self.dt
        vel_profile = np.linspace(start_velocity, target_velocity, int(steps))

        return vel_profile
