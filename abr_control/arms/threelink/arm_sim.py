import numpy as np

from .arm_files.py3LinkArm import pySim


class ArmSim():
    """ An interface for the three-link MapleSim model

    An interface for the three-link MapleSim model that has been exported
    to C and turned into shared libraries using Cython.

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    dt: float, optional (Default: 0.001)
        simulation time step [seconds]
    q_init : numpy.array, optional (Default: None)
        start joint angles [radians]
    """

    def __init__(self, robot_config, dt=.001, q_init=None):

        self.robot_config = robot_config

        self.q = np.zeros(self.robot_config.N_JOINTS)  # joint angles
        self.dq = np.zeros(self.robot_config.N_JOINTS)  # joint_velocities

        if q_init is not None:
            self.q_init = np.zeros(self.robot_config.N_JOINTS*2)
            self.q_init[::2] = q_init
            # TODO: add in ability to set starting velocity, if useful
            # self.q_init[1::2] = dq_init
        else:
            self.q_init = None

        self.dt = dt  # time step

    def connect(self):
        """ Creates the MapleSim model and set up PyGame.
        """

        # stores information returned from maplesim
        self.state = np.zeros(7)
        self.sim = pySim(dt=1e-5)

        self.sim.reset(self.state, self.q_init)
        self._update_state()
        print('Connected to MapleSim model')

    def disconnect(self):
        """ Reset the simulation and close PyGame display.
        """

        self.sim.reset(self.state, self.q_init)
        self._update_state()
        print('MapleSim connection closed...')

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

        dt = self.dt if dt is None else dt
        u = -1 * np.array(u, dtype='float')

        for ii in range(int(np.ceil(dt/1e-5))):
            self.sim.step(self.state, u)
        self._update_state()

    def get_feedback(self):
        """ Return a dictionary of information needed by the controller. """

        return {'q': self.q,
                'dq': self.dq}

    def get_xyz(self, name):
        """ Not available in the MapleSim Interface"""

        raise NotImplementedError("Not an available method" +
                                  "in the MapleSim interface")

    def _position(self):
        """Compute x,y position of the hand
        """

        xy = [self.robot_config.Tx('joint%i' % ii, q=self.q)
              for ii in range(self.robot_config.N_JOINTS)]
        xy = np.vstack([xy, self.robot_config.Tx('EE', q=self.q)])
        self.joints_x = xy[:, 0]
        self.joints_y = xy[:, 1]
        return np.array([self.joints_x, self.joints_y])

    def _update_state(self):
        """Update the local variables"""

        self.t = self.state[0]
        self.q = self.state[1:4]
        self.dq = self.state[4:]
        self._position()
        self.x = np.array([self.joints_x[-1], self.joints_y[-1]])
