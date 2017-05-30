class PathPlanner(object):
    """ Super class for any trajectory planners. """

    def __init__(self, robot_config):

        self.robot_config = robot_config

    def generate(self):
        """ Generate the path for the system to follow """
        raise NotImplementedError

    def next(self):
        """ Return the next target point along the path """
        raise NotImplementedError
