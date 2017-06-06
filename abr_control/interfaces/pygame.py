import numpy as np
import pygame
import pygame.locals


class PyGame():
    """ Set up the PyGame visualization window.

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    arm_sim
    dt: float, optional (Default: 0.001)
        simulation timestep
    q_init: numpy.array, optional (Default: None)
        joint start position [radians]
    on_click: function
        function to call on mouse click, parameters
        are (Display, mouse_x, mouse_y)
    on_keypress : function
        function to call on keypress, parameters
        are (Display, key)
    """

    def __init__(self, robot_config, arm_sim, dt=0.001, q_init=None,
                 on_click=None, on_keypress=None):
        self.robot_config = robot_config
        self.arm_sim = arm_sim

        # set up size of pygame window
        self.width = 642
        self.height = 600
        # calculate centering offset
        self.base_offset = np.array([self.width / 2.0, self.height*.9],
                                    dtype='int')

        self.scaling_term = 105
        line_width = .15 * self.scaling_term

        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.black = (0, 0, 0)
        self.arm_color = (75, 75, 75)
        line_color = (50, 50, 50, 200)  # fourth value is transparency

        self.on_click = on_click
        self.on_keypress = on_keypress

        # NOTE: assuming that the arm lengths are entirely in the x dimension
        # To get the distance between links we add up couples of transforms,
        # because the robot_config.L is defined as sets of [joint_to_com,
        # com_to_joint] distances. We ignore the first offset because it's
        # from the origin to joint 0 and we want joint 0 centered.
        # TODO: make this more robust, for systems with offsets
        #       along several dimensions
        L = []
        for ii in range(int(self.robot_config.L.shape[0] / 2)):
            L.append(np.sum(self.robot_config.L[ii*2:ii*2+2]))
        self.L = np.asarray(L) * self.scaling_term
        # for plotting minimum distance is 1 pixel, otherwise seg fault
        self.L[self.L < 1] = 1

        self.target = None
        self.circles = []
        self.mouse_x = None
        self.mouse_y = None

        # create transparent arm lines
        self.lines_base = []
        for ii in range(len(self.L)):
            self.lines_base.append(pygame.Surface((self.L[ii], line_width),
                                   pygame.SRCALPHA, 32))
            # color in transparent arm lines
            self.lines_base[ii].fill(line_color)

        self.fps = 20  # frames per second
        self.fpsClock = pygame.time.Clock()

    def connect(self):
        """ Create the PyGame display, instantiate the arm simulation.
        """

        self.arm_sim.connect()

        pygame.init()
        self.display = pygame.display.set_mode((self.width, self.height))

        # initialize font
        self.myfont = pygame.font.SysFont("monospace", 15)

        print('Connected to PyGame display')

    def disconnect(self):
        """ Close the PyGame display.
        """

        pygame.quit()
        self.arm_sim.disconnect()
        print('PyGame connection closed...')

    def get_feedback(self):
        """ Return a dictionary of information needed by the controller. """

        return self.arm_sim.get_feedback()

    def send_forces(self, u, dt=None):
        """ Apply the specified torque to the robot

        Apply the specified forces to the robot,
        move the simulation one time step forward, and update
        the PyGame display.

        Parameters
        ----------
        u : numpy.array
            an array of the torques to apply to the robot [Nm]
        dt : float, optional (Default: None)
            time step [seconds]
        """

        # update the arm sim
        self.arm_sim.send_forces(u, dt=dt)
        # update the display
        self._update(self.arm_sim.q)

    def set_target(self, xyz):
        """ Set the position of the target object.

        Parameters
        ----------
        xyz : np.array
            the [x,y,z] location of the target [meters]
        """

        self.target = (xyz[:2] * np.array([1, -1]) *
                       self.scaling_term + self.base_offset)

    def add_circle(self, xyz, radius, color=[0, 0, 100]):
        """ Add an obstacle to the list.

        Parameters
        ----------
        xyz : np.array
            the [x,y,z] location of the target [meters]
        radius : float
            the radius of the obstacle [meters]
        color : list, optional (Default: [0, 0, 100])
            colour of the circle
        """

        circle = list(xyz[:2] * np.array([1, -1]) *
                      self.scaling_term + self.base_offset)
        circle.append(radius * self.scaling_term)
        circle += color
        self.circles.append(circle)

    def get_mousexy(self):
        """ Returns the (x,y) position of the mouse over the display.
        """

        if self.mouse_x is not None and self.mouse_y is not None:
            x = (self.mouse_x - self.base_offset[0]) / self.scaling_term
            y = (self.mouse_y - self.base_offset[1]) / self.scaling_term * -1
            return x, y
        return None

    def get_xyz(self, name):
        """ Not available in the pygame interface"""

        raise NotImplementedError(
            "Not an available method in the PyGame interface")

    def _update(self, q):
        """ Update the arm using the provided joint angles.

        Parameters
        ----------
        q : np.array
            the current joint angles
        """

        self.display.fill(self.white)

        # need to pad q with a 0 for the origin -> joint 0 offset
        q = [0] + list(q)
        # get (x,y) positions of the joints
        joints_x = np.array(np.cumsum([0] + [
            int(self.L[ii] * np.cos(np.sum(q[:ii+1])))
            for ii in range(len(self.L))])) + self.base_offset[0]
        joints_y = np.array(np.cumsum([0] + [
            int(-self.L[ii] * np.sin(np.sum(q[:ii+1])))
            for ii in range(len(self.L))])) + self.base_offset[1]
        points = np.vstack([joints_x, joints_y]).T

        self.lines = []
        self.rects = []
        for ii in range(len(self.L)):
            self.lines.append(pygame.transform.rotozoom(
                self.lines_base[ii],
                np.degrees(np.sum(q[:ii+1])), 1))
            self.rects.append(self.lines[ii].get_rect())

            self.rects[ii].center += np.asarray(points[ii])
            self.rects[ii].center += np.array([
                np.cos(np.sum(q[:ii+1])) * self.L[ii] / 2.0,
                -np.sin(np.sum(q[:ii+1])) * self.L[ii] / 2.0])
            self.rects[ii].center += np.array([
                -self.rects[ii].width / 2.0,
                -self.rects[ii].height / 2.0])

        # draw arm lines
        for ii in range(len(self.L)):
            self.display.blit(self.lines[ii], self.rects[ii])
            # draw circles at joint
            pygame.draw.circle(
                self.display, self.black, points[ii],
                int((len(self.L) - ii) * 10))
            pygame.draw.circle(
                self.display, self.arm_color, points[ii],
                int((len(self.L) - ii) * 5))
        # draw circles
        for circle in self.circles:
            pygame.draw.circle(
                self.display, circle[3:7],  # circle color
                [int(circle[0]), int(circle[1])],
                int(circle[2]))  # circle size
        # draw target
        if self.target is not None:
            pygame.draw.circle(
                self.display, self.red,
                [int(val) for val in self.target], 10)

        # check for events
        for event in pygame.event.get():
            if (event.type == pygame.MOUSEBUTTONDOWN and
                    self.on_click is not None):
                self.mouse_x, self.mouse_y = pygame.mouse.get_pos()
                self.on_click(self, self.mouse_x, self.mouse_y)

            if event.type == pygame.KEYDOWN:
                if (event.key == pygame.K_w and
                        pygame.key.get_mods() & pygame.KMOD_CTRL):
                    pygame.quit()

                elif self.on_keypress is not None:
                    self.on_keypress(self, event.key)

            if event.type == pygame.QUIT:
                pygame.quit()

        # render text
        label = self.myfont.render(
            "Time: %0.3fs" % self.arm_sim.t, 1, (0, 0, 0))
        self.display.blit(label, (10, 10))

        pygame.display.update()
        self.fpsClock.tick(self.fps)
