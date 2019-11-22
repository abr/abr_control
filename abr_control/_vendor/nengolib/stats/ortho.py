import numpy as np

from scipy.linalg import svd

from nengo.dists import UniformHypersphere


def random_orthogonal(d, rng=None):
    """Returns a random orthogonal matrix.

    Parameters
    ----------
    d : ``integer``
        Positive dimension of returned matrix.
    rng : :class:`numpy.random.RandomState` or ``None``, optional
        Random number generator state.

    Returns
    -------
    samples : ``(d, d) np.array``
        Random orthogonal matrix (an orthonormal basis);
        linearly transforms any vector into a uniformly sampled
        vector on the ``d``--ball with the same L2 norm.

    See Also
    --------
    :class:`.ScatteredHypersphere`

    Examples
    --------
    >>> from nengolib.stats import random_orthogonal, sphere
    >>> rng = np.random.RandomState(seed=0)
    >>> u = sphere.sample(1000, 3, rng=rng)
    >>> u[:, 0] = 0
    >>> v = u.dot(random_orthogonal(3, rng=rng))

    >>> import matplotlib.pyplot as plt
    >>> from mpl_toolkits.mplot3d import Axes3D
    >>> ax = plt.subplot(111, projection='3d')
    >>> ax.scatter(*u.T, alpha=.5, label="u")
    >>> ax.scatter(*v.T, alpha=.5, label="v")
    >>> ax.patch.set_facecolor('white')
    >>> ax.set_xlim3d(-1, 1)
    >>> ax.set_ylim3d(-1, 1)
    >>> ax.set_zlim3d(-1, 1)
    >>> plt.legend()
    >>> plt.show()
    """

    rng = np.random if rng is None else rng
    m = UniformHypersphere(surface=True).sample(d, d, rng=rng)
    u, s, v = svd(m)
    return np.dot(u, v)
