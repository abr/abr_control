import numpy as np


class ArmSim():
    """ An interface for a Python implementation of a 2 link arm.

    An interface for the two-link Python model arm.

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    dt: float, optional (Default: 0.001)
        simulation time step [seconds]
    q_init : numpy.array, optional (Default: robot_config.REST_ANGLES)
        start joint angles [radians]
    """

    def __init__(self, robot_config, dt=.001, q_init=None):

        self.robot_config = robot_config

        self.q_init = (q_init if q_init is not None else
                       self.robot_config.REST_ANGLES)
        self.reset()

        M = self.robot_config._M_LINKS
        L = []
        for ii in range(int(self.robot_config.L.shape[0] / 2)):
            L.append(np.sum(self.robot_config.L[ii*2:ii*2+2]))

        # compute non changing constants
        self.K1 = ((1/3. * M[1][0, 0] + M[2][0, 0]) * L[1]**2. +
                   1/3. * M[2][0, 0] * L[2]**2.)
        self.K2 = M[2][0, 0] * L[1] * L[2]
        self.K3 = 1/3. * M[2][0, 0] * L[2]**2.
        self.K4 = 1/2. * M[2][0, 0] * L[1] * L[2]

        self.dt = dt  # time step
        self.t = 0.0  # time

    def connect(self):
        """ Reset the state of the system.
        """
        self.reset()
        self._update_state()
        # NOTE: This is kind of a meaningless comment, not sure if worth having
        print('Connected to Python model')

    def disconnect(self):
        """ Reset the simulation and close PyGame display.
        """

        self.reset()
        self._update_state()
        # NOTE: This is kind of a meaningless comment, not sure if worth having
        print('Python model connection closed...')

    def get_feedback(self):
        """ Return a dictionary of information needed by the controller. """

        return {'q': self.q,
                'dq': self.dq}

    def get_xyz(self, name):
        raise NotImplementedError("Not an available method" +
                                  "in the MapleSim interface")

    def send_forces(self, u, dt=None):
        """ Apply the specified forces to the robot,
        move the simulation one time step forward, and update
        the plot.

        Parameters
        ----------
        u : numpy.array
            an array of the torques to apply to the robot [Nm]
        dt : float, optional (Default: None)
            time step [seconds]
        """

        self._step(u, dt)

    def reset(self):
        """ Resets the state of the arm to starting conditions.
        """

        self.q = np.copy(self.q_init)
        self.dq = np.zeros(self.q.shape)

    def _position(self):
        """ Compute x,y position of the hand
        """

        xy = [self.robot_config.Tx('joint%i' % ii, q=self.q)
              for ii in range(self.robot_config.N_JOINTS)]
        xy = np.vstack([xy, self.robot_config.Tx('EE', q=self.q)])
        self.joints_x = xy[:, 0]
        self.joints_y = xy[:, 1]
        return np.array([self.joints_x, self.joints_y])

    def _step(self, u, dt=None):
        """ Simulate the system one time step

        Parameters
        ----------
        u : numpy.array
            an array of the torques to apply to the robot
        dt : float, optional (Default: self.dt)
            time step [seconds]
        """

        dt = self.dt if dt is None else dt

        # equations solved for angles
        C2 = np.cos(self.q[1])
        S2 = np.sin(self.q[1])
        M11 = (self.K1 + self.K2 * C2)
        M12 = (self.K3 + self.K4 * C2)
        M21 = M12
        M22 = self.K3
        H1 = (-self.K2 * S2 * self.dq[0] * self.dq[1] -
              1.0/2.0 * self.K2 * S2 * self.dq[1]**2.0)
        H2 = 1.0/2.0 * self.K2 * S2 * self.dq[0]**2.0

        ddq1 = ((H2*M11 - H1*M21 - M11*u[1] + M21*u[0]) /
                (M12**2. - M11*M22))
        ddq0 = (-H2 + u[1] - M22*ddq1) / M21
        self.dq += np.array([ddq0, ddq1]) * dt
        self.q += self.dq * dt

        # transfer to next time step
        self.t += self.dt

        self._update_state()

    def _update_state(self):
        """ Update local variables """

        self._position()
        self.x = np.array([self.joints_x[-1], self.joints_y[-1]])
