import numpy as np
import os
import scipy.linalg as sp_linalg

from abr_control.controllers import path_planners

class GenerateTrajectory():
    def __init__(self):
        # converted using the abr_jaco2 Config.Tx('EE', q) function
        #self.start_xyz = np.array([0.006, -0.007, 1.06])

        # using initial ee_xyz value from recorded data
        self.start_xyz = np.array([-0.004, -0.013, 1.183])

    def get_ideal(self, reaching_dt=3, target_xyz=None):
        if target_xyz is None:
            print('ERROR: Must provide target(s)')
        x_track = []
        u_track = []
        # create our point mass system dynamics dx = Ax + Bu
        x = np.hstack([self.start_xyz, np.zeros(3)])  # [x, y, z, dx, dy, dz]
        A = np.array([
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]])
        u = np.array([0, 0, 0])  # [u_x, u_y, u_z]
        B = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]])

        # create our state and control cost matrices
        Qs = np.ones(6) * 10
        # Rs = [0.35, 0.225, 0.5, 0.2, 0.225]
        Rs = np.ones(6) * 1e-3

        # interpolation sampling rate
        dt = 0.005
        timesteps = int(reaching_dt / dt)
        print('time steps: ', timesteps)
        continuous = False

        vmax = 1
        kp = 20
        kv = 6
        lamb = kp / kv

        path = path_planners.SecondOrder(None)
        w = 1e4 / timesteps

        for ii, target in enumerate(target_xyz):
            u = np.zeros(3)
            Q = Qs[ii] * np.eye(6)
            R = Rs[ii] * np.eye(3)
            print('II: ', ii)

            for t in range(timesteps):
                # track trajectory
                x_track.append(np.copy(x))
                u_track.append(np.copy(u))

                temp_target = path.step(
                    y=target, dy=np.zeros(3),
                    target=target, w=w,
                    zeta=2, dt=0.003, threshold=0.05)

                # calculate the position error
                x_tilde = np.array(x[:3] - temp_target[:3])

                # implement velocity limiting
                sat = vmax / (lamb * np.abs(x_tilde))
                if np.any(sat < 1):
                    index = np.argmin(sat)
                    unclipped = kp * x_tilde[index]
                    clipped = kv * vmax * np.sign(x_tilde[index])
                    scale = np.ones(3, dtype='float32') * clipped / unclipped
                    scale[index] = 1
                else:
                    scale = np.ones(3, dtype='float32')

                u = -kv * (x[3:] - temp_target[3:] -
                                    np.clip(sat / scale, 0, 1) *
                                    -lamb * scale * x_tilde)

                # move simulation one time step forward
                dx = np.dot(A, x) + np.dot(B, u)
                x += dx * dt

        u_track = np.array(u_track)
        x_track = np.array(x_track)
        print('xtrack_ideal %isec x %i reaches: '%(reaching_dt, len(target_xyz)), x_track.shape)
        print('utrack_ideal %isec x %i reaches: '%(reaching_dt, len(target_xyz)), u_track.shape)
        np.savez_compressed('proc_data/pd_x_%isec_x_%itargets'
                % (reaching_dt, len(target_xyz)),
                x=x_track[:, :3], dx=x_track[:, 3:])
        np.savez_compressed('proc_data/pd_u_%isec_x_%itargets'
                % (reaching_dt, len(target_xyz)),
                u=u_track)
        np.savez_compressed('proc_data/pd_targets_%isec_x_%itargets'
                % (reaching_dt, len(target_xyz)),
                target_xyz=target_xyz)

        return [u_track, x_track]
