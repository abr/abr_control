import numpy as np
import pygame
import pygame.locals
import sys


class interface(interface.interface):
    """ An interface for Python based simulations, plotted in PyGame.
    """

    def __init__(self, robot_config, arm, dt=.001):
        super(interface, self).__init__(robot_config)

        self.arm = arm

        # set up size of pygame window
        self.width = 642
        self.height = 600
        # calculate centering offset
        self.base_offset = np.array([self.width / 2.0, self.height*.9])

        self.scaling_term = 105
        self.line_width = .15 * scaling_term

        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.black = (0, 0, 0)
        self.arm_color = (75, 75, 75)
        self.line_color = (50, 50, 50, 200) # fourth value is transparency

    def connect(self):

        self.upperarm_length = self.arm.L[0] * scaling_term
        self.forearm_length = self.arm.L[1] * scaling_term
        self.hand_length = self.arm.L[2] * scaling_term

        # create transparent arm lines
        self.line_upperarm_base = pygame.Surface((upperarm_length, line_width),
                pygame.SRCALPHA, 32)
        self.line_forearm_base = pygame.Surface((forearm_length, line_width),
                pygame.SRCALPHA, 32)
        self.line_hand_base = pygame.Surface((hand_length, line_width),
                pygame.SRCALPHA, 32)

        # color in transparent arm lines
        self.line_upperarm_base.fill(line_color)
        self.line_forearm_base.fill(line_color)
        self.line_hand_base.fill(line_color)

        fps = 20 # frames per second
        fpsClock = pygame.time.Clock()

        pygame.init()
        self.display = pygame.display.set_mode((self.width, self.height))

    def apply_u(self, u):

        q = self.arm.q

        self.display.fill(white)

        # get (x,y) positions of the joints
        x,y = self.arm.position()
        points = [(int(a * scaling_term + self.base_offset[0]),
                    int(-b * scaling_term + self.base_offset[1]))
                    for a,b in zip(x,y)]

        # transparent upperarm line
        line_upperarm = pygame.transform.rotozoom(
            self.line_upperarm_base,
            np.degrees(q[0]), 1)
        rect_upperarm = line_upperarm.get_rect()

        # transparent forearm line
        line_forearm = pygame.transform.rotozoom(
            self.line_forearm_base,
            np.degrees(q[0] + q[1]), 1)
        rect_forearm = line_forearm.get_rect()

        # transparent hand line
        line_hand = pygame.transform.rotozoom(
            self.line_hand_base,
            np.degrees(q[0] + q[1] + q[2]), 1)
        rect_hand = line_hand.get_rect()

        # draw things!
        self.display.blit(background, (0,0)) # draw on the background

        for trail in self.trail_data:
            pygame.draw.aalines(self.display, black, False, trail, True)

        # draw transparent arm lines
        self.display.blit(line_upperarm, rect_upperarm)
        self.display.blit(line_forearm, rect_forearm)
        self.display.blit(line_hand, rect_hand)

        # draw circles at shoulder
        pygame.draw.circle(self.display, black, points[0], 30)
        pygame.draw.circle(self.display, arm_color, points[0], 12)

        # draw circles at elbow
        pygame.draw.circle(self.display, black, points[1], 20)
        pygame.draw.circle(self.display, arm_color, points[1], 7)

        # draw circles at wrist
        pygame.draw.circle(self.display, black, points[2], 15)
        pygame.draw.circle(self.display, arm_color, points[2], 5)

        # draw target
        pygame.draw.circle(self.display, red, [int(val) for val in self.target], 10)

        pygame.display.update()
        fpsClock.tick(fps)

    def disconnect(self):
        """ Stop and reset the simulation. """
            pygame.quit()
            sys.exit()

    def get_feedback(self):
        """ Return a dictionary of information needed by the controller. """
        return {'q': self.arm.q,
                'dq': self.arm.dq}

    def set_target(self, xyz):
        """ Set the position of the target object.

        xyz np.array: the [x,y,z] location of the target (in meters)
        """
        self.target = xyz * np.array([1, -1]) * scaling_term + self.base_offset
