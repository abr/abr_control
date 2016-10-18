import numpy as np


class controller:
    """ Implements an trajectory controller over on top of a given
    point to point control system.
    """

    def __init__(self, controller):

        self.controller = controller

        self.target_state = None  # for tracking target changes

    def control(self, q, target_state, n_timesteps=200,
                endpoint_name='EE', **kwargs):
        """ Generates the control signal.
        The trajectory controller checks to see if the target has changed,
        if it has, it generates a new desired trajectory that moves the system
        from the current state to the target state in n_timesteps. The
        trajectory will move from the current state to the target in a
        straight line.

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_state np.array: the target [pos, vel] for the end-effector
        n_timesteps int: the number of time steps to reach the target
        """

        # check for change in target
        if np.any(target_state != self.target_state):
            # generate a new desired trajectory
            self.trajectory = np.zeros((n_timesteps, 6))
            # generate desired positions
            xyz = self.controller.robot_config.T(endpoint_name, q)
            for ii in range(3):
                self.trajectory[:, ii] = np.linspace(xyz[ii],
                                                     target_state[ii],
                                                     n_timesteps)
                self.trajectory[:-1, ii+3] = (
                    np.diff(self.trajectory[:, ii]) / 0.001)
            self.target_state = np.copy(target_state)
            # reset trajectory index
            self.n = 0
            print(self.trajectory)

        target_state = (self.trajectory[self.n]
                        if self.n < n_timesteps else target_state)
        self.n += 1
        print('n: ', self.n)

        return self.controller.control(
            q=q, target_state=target_state, **kwargs)
