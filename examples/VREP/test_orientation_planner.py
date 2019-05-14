"""
Running operational space control using VREP. The controller will
move the end-effector to the target object's orientation.
"""
import numpy as np
from abr_analyze.plotting import Draw3dData, MakeGif
from abr_analyze.paths import cache_dir, figures_dir
from abr_control.utils import transformations
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d  # pylint: disable=W0611
from random import gauss

def make_rand_vector(dims):
    vec = np.random.random(3)
    vec /= np.linalg.norm(vec)
    return list(vec)

gif = MakeGif()
fig_cache = gif.prep_fig_cache()

target_quat =  [0] + make_rand_vector(3)
vector = [0] + make_rand_vector(3)
start_quat = vector.copy()

def rotate_vector(vector, q):
    q_conj = [q[0], -1*q[1], -1*q[2], -1*q[3]]
    ans = transformations.quaternion_multiply(
        transformations.quaternion_multiply(q, vector), q_conj)
    print(ans)
    return ans

try:
    fig = plt.figure()
    ax = []
    for ii in range(4):
        ax.append(fig.add_subplot(2, 2, ii+1, projection='3d'))

    fraction = np.linspace(0, 1, 100)
    elev = [None, 90, 0, 90]
    azim = [None, 0, 90, 90]
    dr_err = []

    for ii in range(len(fraction)):
        q = transformations.quaternion_slerp(
            quat0=start_quat, quat1=target_quat, fraction=fraction[ii])

        v1 = rotate_vector(vector, q)
        # v2 = rotate_vector(vector, start_quat)
        # v3 = rotate_vector(vector, target_quat)

        dr = (q[0] * np.asarray(target_quat)[1:] -
                np.asarray(target_quat)[0] * q[1:] -
                np.cross(np.asarray(target_quat)[1:], q[1:]))
        dr_err.append(np.copy(np.linalg.norm(dr, 2)))


        for jj, a in enumerate(ax):
            # axis
            a.quiver(0, 0, 0, 0, 0, 1, length=1, color='b')
            a.quiver(0, 0, 0, 0, 1, 0, length=1, color='b')
            a.quiver(0, 0, 0, 1, 0, 0, length=1, color='b')

            # path planner
            a.quiver(0, 0, 0, q[1], q[2], q[3], length=1, normalize=True,
                     color='k')
            # start orientation
            a.quiver(
                0, 0, 0,
                vector[1], vector[2], vector[3],
                length=1,
                normalize=True,
                color='r')
            # target orientation
            a.quiver(
                0, 0, 0,
                target_quat[1], target_quat[2], target_quat[3],
                length=1,
                normalize=True,
                color='g')

            # labelling and things
            a.set_xticklabels([])
            a.set_yticklabels([])
            a.set_zticklabels([])

            a.set_xlim(-1, 1)
            a.set_ylim(-1, 1)
            a.set_zlim(-1, 1)

            a.set_xlabel('X')
            a.set_ylabel('Y')
            a.set_zlabel('Z')

            a.view_init(elev=elev[jj], azim=azim[jj])

        plt.savefig('%s/%04d' %(fig_cache, ii))
        for a in ax:
            a.clear()

finally:
    # stop and reset the simulation
    print('Simulation terminated...')
    plt.figure()
    plt.plot(dr_err)
    plt.show()
    gif.create(fig_loc=fig_cache,
                save_loc=figures_dir,
                save_name='orientation_planner',
                delay=5, res=[1920,1080])

    # ee_angles_track = np.array(ee_angles_track)
    # target_angles_track = np.array(target_angles_track)
    #
    # if ee_angles_track.shape[0] > 0:
    #     # plot distance from target and 3D trajectory
    #
    #     plt.figure()
    #     plt.plot(ee_angles_track)
    #     plt.gca().set_prop_cycle(None)
    #     plt.plot(target_angles_track, '--')
    #     plt.ylabel('3D orientation (rad)')
    #     plt.xlabel('Time (s)')
    #     plt.tight_layout()
    #     plt.show()
