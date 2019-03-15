"""
"""
import numpy as np
import random

class Target():
    """
    """
    def __init__(redis_server=None, robot_config=None):
        """
        Sets origin offset and passes in redis server connection

        Parameter
        ----------
        redis_server: redis.StrictRedis, Optional (Default: None)
            must be passed in to ge targets from camera
        robot_config: Config.robot_config, Optional (Default: None)
            configuration file of the arm being used, must be passed in for
            most functions
        """
        #self.origin_offset = origin_offset

    def random_target(r=[0.5,1], theta=[0, 6.28], phi=[2.07, 4.21], n_points=1,
            min_separation=0.01, max_separation=1.0, constant_r=False, base=None):
        #TODO: need to handle case where working area crosses zero point
        # can look at avoid joint limits, hand to handle similarly there
        """
        Returns a random cartesian target based on a provided radius and angle
        range of the reachable sphere of the arm

        Parameters
        ----------
        r: list of floats, Optional (Default: [0.5,1])
            radius range from origin to generate target within [meters]
        theta: list of floats, Optional (Default: [0, 6.28])
            the angle range about the origin (parallel with ground) to set
            targets within [radians]
        phi: list of floats, Optional (Default: [2.07, 4.21])
            the angle range perpendicular to the groun to set targets within
            [radians]
            Cannot be from 0-2pi because would result in situations where arm
            reaches through itself or the ground
        n_points: int, Optional (Default: 1)
            the number of xyz points to return
        min_separation: float, Optional (Default: 0.01)
            the minimum distance between adjacent targets [meters]
        max_separation: float, Optional (Default: 1.0)
            the maximum distance between adjacent targets [meters]
        constant_r: boolean, Optional (Default: False)
            True to maintain the same radius for all n_points, this will keep
            the targets on the surface of the same sphere
        """
        assert min_separation <= max_separation, (
                "Minumum (%.2f) separation must be <= Maximum (%.2f)"
                %(min_separation, max_separation))

        points = []
        for ii in range(0, n_points):
            threshold_met = False
            while threshold_met is False:
                if ii == 0 and base is not None:
                    x = base[0]
                    y = base[1]
                    z = base[2]
                    r_0 = np.sqrt(np.sum(
                            (np.array([x,y,z]) - np.array([0,0,0]))**2))
                    threshold_met = True
                else:
                    if ii > 0 and constant_r is True:
                        r = r_0
                    else:
                        r_0 = random.uniform(r[0], r[1])

                    theta_0 = random.uniform(theta[0], theta[1])
                    phi_0 = random.uniform(phi[0], phi[1])

                    # phi mirrors points along z, so if we want to restrict an area based
                    # on a range in theta_0, we need to add pi to theta_0 if phi is greater
                    # than pi
                    if phi_0 > 3.14:
                        theta_0+=3.14
                    # convert from spherical coordinates to cartesian
                    x = r_0 * np.sin(phi_0) * np.cos(theta_0)
                    y = r_0 * np.sin(phi_0) * np.sin(theta_0)
                    # shift phi_0 by Pi to shift origin to be above ground
                    z = r_0 * np.cos(phi_0 + 3.14)
                    if ii == 0:
                        threshold_met = True
                    else:
                        dist = np.sqrt(np.sum(
                            (np.array([x,y,z]) - np.array(points[-1]))**2))
                        if dist < max_separation and dist > min_separation:
                            threshold_met = True

            points.append(np.copy([x, y, z]))
        return points

    def get_target_from_camera(self):
        """
        Reads target from redis server with the key 'target_xyz_robot_coords'
        and returns a target transformed to the arms coordinates

        The transform from camera to robot origin needs to be added to the
        robot config. To see an example look in the abr_jaco2 repo under
        abr_jaco2/abr_jaco2/config/config.py and see the Torgcam transform

        """
        # read from server
        camera_xyz = self.redis_server.get('target_xyz').decode('ascii')
        # if the target has changed, recalculate things
        camera_xyz = np.array([float(val) for val in camera_xyz.split()])
        # transform from camera to robot reference frame
        target_xyz = self.robot_config.Tx(
            'camera', x=camera_xyz, q=np.zeros(6))

        self.redis_server.set(
            'target_xyz_robot_coords', '%.3f %.3f %.3f' % tuple(target_xyz))

        return target_xyz

    def normalize_target(self, target, max_reaching_distance=0.9, origin_offset=[0,0,0]):
        """
        Limits the distance from origin to target to the specified magnitude in
        meters

        Parameters
        ----------
        target: list of floats
            cartesian coordinates of target in meters [x, y, z]
        max_reaching_distance: float, Optional (Default: 0.9)
            the maximum reaching distance from the origin [meters]
        origin_offset: list of floats, Optional (Default: [0, 0, 0])
            the offset from the arms origin. This is usually done if the first
            link rotates and the second joint is the pivoting point (shoulder).
            It may help to think of this as the point that would be in the
            center of the sphere that the arm can reach within. For ex: look at
            joints 0 and 1 of the kinova jaco2.
        """
        # set it so that target is not too far from shoulder
        #joint1_offset = np.array([0, 0, 0.273])
        joint1_offset = np.array(origin_offset)
        norm = np.linalg.norm(target - joint1_offset)
        if norm > max_reaching_distance:
            target = ((target - joint1_offset) / norm) * max_reaching_distance + joint1_offset
        return target
