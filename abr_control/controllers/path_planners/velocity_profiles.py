"""
Functions that return a 1D array of velocities from a desired start to target velocity
"""
import numpy as np
class VelProf():
    def __init__(self, dt):
        self.dt = dt
    def generate(self, start_velocity, target_velocity):
        raise NotImplementedError

class Gaussian(VelProf):
    def __init__(self, dt, acceleration, n_sigma=3):
        """
        Parameters
        ----------
        dt: float
            timestep in seconds
        a: float
            the acceleration that defines our velocity curve
        n_sigma: int, Optional (Default: 3)
            how many standard deviations of the gaussian function to use for the velocity
            profile. The defaul value of 3 gives a smooth acceleration and deceleration.
            A slower ramp up can be achieved with a larger sigma, and a faster ramp up by
            decreasing sigma.
        """
        self.acceleration = acceleration
        self.n_sigma = n_sigma

        super().__init__(dt=dt)

    def generate(self, start_velocity, target_velocity):
        """
        generates the left half of the gaussian curve with 3std, with sigma determined
        by the timestep and max_a
        """
        # calculate the time needed to reach our maximum velocity from vel
        ramp_up_time = (target_velocity-start_velocity)/self.acceleration

        # Amplitude of Gaussian is max speed, get our sigma from here
        s = 1/ ((target_velocity-start_velocity) * np.sqrt(np.pi*2))

        # Go 3 standard deviations so our tail ends get close to zero
        u = self.n_sigma*s

        # We are generating the left half of the gaussian, so generate our
        # x values from 0 to the mean, with the steps determined by our
        # ramp up time (which itself is determined by max_a)
        x = np.linspace(0, u, int(ramp_up_time/self.dt))

        # Get our gaussian y values, which is our normalized velocity profile
        vel_profile = 1 * (
                1/(s*np.sqrt(2*np.pi)) * np.exp(-0.5*((x-u)/s)**2)
        )

        # Since the gaussian goes off to +/- infinity, we need to shift our
        # curve down so that it start_velocitys at zero
        vel_profile -= vel_profile[0]

        # scale back up so we reach our target v at the end of the curve
        vel_profile *= ((target_velocity-start_velocity) / vel_profile[-1])

        # add to our baseline starting velocity
        vel_profile += start_velocity

        return vel_profile

class Linear(VelProf):
    def __init__(self, dt, acceleration):
        self.acceleration = acceleration

        super().__init__(dt=dt)

    def generate(self, start_velocity, target_velocity):
        vdiff = target_velocity - start_velocity
        t = vdiff/self.acceleration
        steps = t/self.dt
        vel_profile = np.linspace(start_velocity, target_velocity, int(steps))

        return vel_profile


class SecondOrderFilter(VelProf):
    def __init__(self, dt, acceleration, zeta=1.0, w=1e4, threshold=0.02):
        self.acceleration = acceleration
        self.dt = dt
        self.zeta = zeta
        self.w = w
        self.threshold = threshold

        super().__init__(dt=dt)

    def generate(self, start_velocity, target_velocity):
        position = np.zeros(3)
        target_position = np.ones(3)

        n_timesteps = int(((target_velocity-start_velocity)/self.acceleration)/self.dt)
        print(n_timesteps)
        w = self.w/n_timesteps

        velocity = start_velocity
        # pos_path = []
        vel_path = []

        for ii in range(0, n_timesteps):
            if np.linalg.norm(position - target_position) < self.threshold:
                # if within a threshold distance, reduce the filter effect
                # NOTE: this is a ad-hoc method of improving performance at
                # short distances
                w *= 3

            accel = w ** 2 * target_position - velocity * self.zeta * w - position * w ** 2
            velocity = velocity + accel * self.dt
            position = position + velocity * self.dt

            # pos_path.append(position)
            vel_path.append(velocity)
        return np.asarray(vel_path)





