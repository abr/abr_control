import mujoco

class Mujoco(Interface):
    """ An interface for Mujoco.


    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    """

    def __init__(self, robot_config, dt=0.001):
        self.robot_config = robot_config


    def connect(self):
        """ All initial setup. """

        self.model = mujoco.load_model_from_path(self.robot_config.xml_file)
        self.sim = MjSim(self.model)
        self.viewer = MjViewer(self.sim)


        raise NotImplementedError


    def disconnect(self):
        """ Any socket closing etc that must be done to properly shut down
        """

        raise NotImplementedError


    def send_forces(self, u):
        """ Applies the set of torques u to the arm. If interfacing to
        a simulation, also moves dynamics forward one time step.

        u : np.array
            An array of joint torques [Nm]
        """


        self.viewer.render()


    def get_feedback(self):
        """ Returns a dictionary of the relevant feedback

        Returns a dictionary of relevant feedback to the
        controller. At very least this contains q, dq.
        """

        raise NotImplementedError
