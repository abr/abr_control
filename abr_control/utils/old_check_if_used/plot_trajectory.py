import numpy as np
             ['no_weight', 'chip', 'gradual_wear8_5', 50, 1,'chip_testing'],
             ['no_weight', 'chip', 'gradual_wear8_5', 50, 1,'chip_testing'],
import os
import scipy.linalg as sp_linalg

from abr_control.controllers import path_planners

#TODO: save pdf results to cache_dir
class PlotTrajectory():
    def __init__(self):
        pass
    def get_recorded(self, test_name, num_sessions, num_runs, session_to_plot,
            run_to_plot):
        u_track = []
        x_track = []

        ee_xyz_rec = np.load(
            'proc_data/%s/ee_xyz_%i_x_%iruns_dt0.0050_%itargets_x_%isec_%s.npz'%
            (test_name, num_sessions, num_runs, self.n_targets,
                self.reaching_time_per_target, test_name))['parameter']

        print('ee: ', ee_xyz_rec.shape)
        q_rec = np.load(
            'proc_data/%s/q_%i_x_%iruns_dt0.0050_%itargets_x_%isec_%s.npz'%
            (test_name, num_sessions, num_runs, self.n_targets,
                self.reaching_time_per_target, test_name))['parameter']

        dq_rec = np.load(
            'proc_data/%s/dq_%i_x_%iruns_dt0.0050_%itargets_x_%isec_%s.npz'%
            (test_name, num_sessions, num_runs, self.n_targets,
                self.reaching_time_per_target, test_name))['parameter']

        u_rec = np.load(
            'proc_data/%s/u_%i_x_%iruns_dt0.0050_%itargets_x_%isec_%s.npz'%
            (test_name, num_sessions, num_runs, self.n_targets,
                self.reaching_time_per_target, test_name))['parameter']

        ee_xyz_rec = ee_xyz_rec[session_to_plot,run_to_plot,:,:].T
        q_rec = q_rec[session_to_plot,run_to_plot,:,:].T
        dq_rec = dq_rec[session_to_plot,run_to_plot,:,:].T
        u_rec = u_rec[session_to_plot,run_to_plot,:,:].T

        for t in range(0, len(u_rec)):
            vel = np.dot(self.robot_config.J('EE',q_rec[t]),dq_rec[t])
            x_track.append(np.copy(
              [ee_xyz_rec[t,0], ee_xyz_rec[t,1], ee_xyz_rec[t,2],
                vel[0], vel[1], vel[2]]))
        return [u_rec, x_track]

    def get_trajectory_plot(self, target_xyz, reaching_t, session_to_plot=0,
            run_to_plot=49, test_set_name=None, use_recorded=True,
            show_plot=True):

        self.n_targets = len(target_xyz)
        self.reaching_time_per_target = reaching_t
        u_track_ideal = np.load('proc_data/pd_u_%isec_x_%itargets.npz'%
                                (reaching_t, len(target_xyz)))['u']
        print('u: ', np.array(u_track_ideal).shape)
        x_track_ideal = (np.array(np.hstack(
                            [np.load('proc_data/pd_x_%isec_x_%itargets.npz'%
                                     (reaching_t, len(target_xyz)))['x'],
                             np.load('proc_data/pd_x_%isec_x_%itargets.npz'%
                                     (reaching_t, len(target_xyz)))['dx']])))
        print('x: ', x_track_ideal.shape)
        target_xyz=np.array(np.load('proc_data/pd_targets_%isec_x_%itargets.npz'%
                                    (reaching_t, len(target_xyz)))['target_xyz'])
        print('targets: ', target_xyz.shape)

        test_info = np.load('proc_data/%s.npz'%test_set_name)['test_info']
        test_info = np.array(test_info)

        colors = ['k', 'b', 'g', 'r', 'y', 'm', 'c']
        dash_colors = ['k--', 'b--', 'g--', 'r--', 'y--', 'm--', 'c--']

        if use_recorded:
            if test_set_name is None:
                print('ERROR: MUST PROVIDE TEST SET NAME TO PLOT RECORDED'
                      + ' DATA')
            import abr_jaco2
            self.robot_config = abr_jaco2.Config(
                use_cython=True, hand_attached=True)

            rec_u_track = []
            rec_x_track = []

            for ii in range(0, len(test_info)):
                backend = test_info[ii,1]
                test_name = test_info[ii,2]
                num_runs = int(test_info[ii,3])
                num_sessions = int(test_info[ii,4])

                [u_track, x_track] = self.get_recorded(test_name=test_name,
                        num_sessions=num_sessions, num_runs=num_runs,
                        session_to_plot=session_to_plot,
                        run_to_plot=run_to_plot)
                rec_u_track.append(np.copy(u_track))
                rec_x_track.append(np.copy(x_track))
            rec_u_track = np.array(rec_u_track)
            rec_x_track = np.array(rec_x_track)

            print('x_track_rec: ', rec_x_track.shape)
            print('u_track_rec: ', rec_u_track.shape)

        # Plot Results
        from mpl_toolkits.mplot3d import axes3d
        import matplotlib
        matplotlib.use("TKAgg")
        import matplotlib.pyplot as plt

        if not os.path.exists('figures'):
            os.makedirs('figures')
        if not os.path.exists('figures/%s'%test_set_name):
            os.makedirs('figures/%s'%test_set_name)

        plt.figure(figsize=(9, 3))
        ax0 = plt.subplot(1, 3, 1)
        plt.grid()
        plt.plot(target_xyz[0, 0], target_xyz[0, 1], 'rx', mew=3)
        plt.plot(x_track_ideal[:, 0], x_track_ideal[:, 1], 'r--', mew=2)
        plt.ylim([-.35, .4])
        plt.xlim([-.05, .75])
        plt.xlabel('x')
        plt.ylabel('y')
        ax0.set_aspect('equal')

        ax1 = plt.subplot(1, 3, 2)
        plt.grid()
        plt.plot(target_xyz[0, 0], target_xyz[0, 2], 'rx', mew=3)
        plt.plot(x_track_ideal[:, 0], x_track_ideal[:, 2], 'r--', mew=2)
        plt.ylim([.6, 1.35])
        plt.xlim([-.05, .75])
        plt.xlabel('x')
        plt.ylabel('z')
        plt.title('2D plots of trajectories', y=1.1)
        ax1.set_aspect('equal')

        ax2 = plt.subplot(1, 3, 3)
        plt.grid()
        plt.plot(target_xyz[0, 1], target_xyz[0, 2], 'rx', mew=3)
        plt.plot(x_track_ideal[:, 1], x_track_ideal[:, 2], 'r--', mew=2)
        plt.xlim([-.35, .35])
        plt.ylim([.6, 1.3])
        plt.xlabel('y')
        plt.ylabel('z')
        ax2.set_aspect('equal')

        for ii in range(0, len(test_info)):

            ax0.plot(rec_x_track[ii, :, 0], rec_x_track[ii, :, 1], colors[ii], lw=2)

            ax1.plot(rec_x_track[ii, :, 0], rec_x_track[ii, :, 2], colors[ii], lw=2)

            ax2.plot(rec_x_track[ii, :, 1], rec_x_track[ii, :, 2], colors[ii], lw=2)

        plt.tight_layout()
        plt.savefig('2D_trajectory.pdf')
        if show_plot:
            plt.show()

        # for ii in range(0, len(test_info)):
            # fig = plt.figure(figsize=(7,10))
            # plt.title('%s_%s_Trajectory_run%i.pdf'%(test_info[ii,2],
            #     test_info[ii,1], run_to_plot))
            # ax1 = fig.add_subplot(411, projection='3d')
            # plt.plot(x_track_ideal[:, 0], x_track_ideal[:, 1],
            #         x_track_ideal[:, 2], colors[0], label='Ideal Trajectory')
            # if use_recorded:
            #     plt.plot(rec_x_track[ii, :, 0],
            #              rec_x_track[ii, :, 1],
            #              rec_x_track[ii, :, 2],
            #              colors[3],
            #              label=test_info[ii,2])
            # for target in target_xyz:
            #     plt.plot([target[0]], [target[1]], [target[2]], 'rx', mew=5)
            # plt.legend()
            # ax1.view_init(azim=45)
            # ax1.set_aspect('equal')
            #
            # ax2 = fig.add_subplot(412)
            # if use_recorded:
            #     plt.plot(rec_x_track[ii, :, 0], colors[0], label='X')
            #     plt.plot(rec_x_track[ii, :, 1], colors[1], label='Y')
            #     plt.plot(rec_x_track[ii, :, 2], colors[2], label='Z')
            # plt.plot(x_track_ideal[:, 0], dash_colors[0], label='X ideal')
            # plt.plot(x_track_ideal[:, 1], dash_colors[1], label='Y ideal')
            # plt.plot(x_track_ideal[:, 2], dash_colors[2], label='Z ideal')
            # plt.ylabel('Positions (m)')
            # plt.legend()
            #
            # ax3 = fig.add_subplot(413)
            # if use_recorded:
            #     plt.plot(rec_x_track[ii, :, 3], colors[3], label='Vx')
            #     plt.plot(rec_x_track[ii, :, 4], colors[4], label='Vy')
            #     plt.plot(rec_x_track[ii, :, 5], colors[5], label='Vz')
            # plt.plot(x_track_ideal[:, 3], dash_colors[3], label='Vx ideal')
            # plt.plot(x_track_ideal[:, 4], dash_colors[4], label='Vy ideal')
            # plt.plot(x_track_ideal[:, 5], dash_colors[5], label='Vz ideal')
            # plt.ylabel('Velocities (m/s)')
            # plt.legend()
            #
            # ax4 = fig.add_subplot(414)
            # # if use_recorded:
            # #     plt.plot(rec_u_track[ii], label=test_info[ii,2])
            # # plt.plot(u_track_ideal, '--', label='ideal')
            # # plt.ylabel('Nm')
            # # plt.legend()
            # ax4.axvline(x=run_to_plot, color='r')
            # ax4.set_xlim([0,100])
            #
            # plt.tight_layout()
            # plt.savefig('figures/%s/%s_Trajectory_run%03d.pdf'%(
            #     test_set_name, test_info[ii,2], run_to_plot))
            # if show_plot:
            #     plt.show()
