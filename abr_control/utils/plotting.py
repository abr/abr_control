import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import seaborn


def plot_trajectory(ee_path, target_path, save_file_name=None):
    """ 3D plot of the end-effector and target trajectory

    ee_path np.array: the ee trajectory [[x0,y0,z0],...,[xN,yN,zN]]
    target_path np.array: the target trajectory [[x0,y0,z0],...,[xN,yN,zN]]
    save_file_name string: saves the figure with this name
    """

    ee_path = np.asarray(ee_path)
    target_path = np.asarray(target_path)

    fig = plt.figure(figsize=(6, 6))

    ax = fig.add_subplot(1, 1, 1, projection='3d')
    # plot start point of hand
    ax.plot([ee_path[0, 0]], [ee_path[0, 1]],
            [ee_path[0, 2]], 'bx', mew=10)
    # plot trajectory of hand
    ax.plot(ee_path[:, 0], ee_path[:, 1],
            ee_path[:, 2])
    # plot trajectory of target
    ax.plot(target_path[:, 0], target_path[:, 1],
            target_path[:, 2], 'rx', mew=10)

    ax.set_xlim([-1, 1])
    ax.set_ylim([-.5, .5])
    ax.set_zlim([0, 1])
    ax.legend()

    plt.tight_layout()

    if save_file_name is not None:
        plt.savefig(save_file_name)

    plt.show()
