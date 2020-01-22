import warnings

import numpy as np
from scipy.special import beta, betainc, betaincinv

from nengo.dists import Distribution, UniformHypersphere

from abr_control._vendor.nengolib.stats.ortho import random_orthogonal


def spherical_transform(samples):
    """Map samples from the ``[0, 1]``--cube onto the hypersphere.

    Applies the `inverse transform method` to the distribution
    :class:`.SphericalCoords` to map uniform samples from the ``[0, 1]``--cube
    onto the surface of the hypersphere. [#]_

    Parameters
    ----------
    samples : ``(n, d) array_like``
        ``n`` uniform samples from the d-dimensional ``[0, 1]``--cube.

    Returns
    -------
    mapped_samples : ``(n, d+1) np.array``
        ``n`` uniform samples from the ``d``--dimensional sphere
        (Euclidean dimension of ``d+1``).

    See Also
    --------
    :class:`.Rd`
    :class:`.Sobol`
    :class:`.ScatteredHypersphere`
    :class:`.SphericalCoords`

    References
    ----------
    .. [#] K.-T. Fang and Y. Wang, Number-Theoretic Methods in Statistics.
       Chapman & Hall, 1994.

    Examples
    --------
    >>> from nengolib.stats import spherical_transform

    In the simplest case, we can map a one-dimensional uniform distribution
    onto a circle:

    >>> line = np.linspace(0, 1, 20)
    >>> mapped = spherical_transform(line)

    >>> import matplotlib.pyplot as plt
    >>> plt.figure(figsize=(6, 3))
    >>> plt.subplot(121)
    >>> plt.title("Original")
    >>> plt.scatter(line, np.zeros_like(line), s=30)
    >>> plt.subplot(122)
    >>> plt.title("Mapped")
    >>> plt.scatter(*mapped.T, s=25)
    >>> plt.show()

    This technique also generalizes to less trivial situations, for instance
    mapping a square onto a sphere:

    >>> square = np.asarray([[x, y] for x in np.linspace(0, 1, 50)
    >>>                             for y in np.linspace(0, 1, 10)])
    >>> mapped = spherical_transform(square)

    >>> from mpl_toolkits.mplot3d import Axes3D
    >>> plt.figure(figsize=(6, 3))
    >>> plt.subplot(121)
    >>> plt.title("Original")
    >>> plt.scatter(*square.T, s=15)
    >>> ax = plt.subplot(122, projection='3d')
    >>> ax.set_title("Mapped").set_y(1.)
    >>> ax.patch.set_facecolor('white')
    >>> ax.set_xlim3d(-1, 1)
    >>> ax.set_ylim3d(-1, 1)
    >>> ax.set_zlim3d(-1, 1)
    >>> ax.scatter(*mapped.T, s=15)
    >>> plt.show()
    """

    samples = np.asarray(samples)
    samples = samples[:, None] if samples.ndim == 1 else samples
    coords = np.empty_like(samples)
    n, d = coords.shape

    # inverse transform method (section 1.5.2)
    for j in range(d):
        coords[:, j] = SphericalCoords(d - j).ppf(samples[:, j])

    # spherical coordinate transform
    mapped = np.ones((n, d + 1))
    i = np.ones(d)
    i[-1] = 2.0
    s = np.sin(i[None, :] * np.pi * coords)
    c = np.cos(i[None, :] * np.pi * coords)
    mapped[:, 1:] = np.cumprod(s, axis=1)
    mapped[:, :-1] *= c
    return mapped


class SphericalCoords(Distribution):
    """Spherical coordinates for inverse transform method.

    This is used to map the hypercube onto the hypersphere and hyperball. [#]_

    Parameters
    ----------
    m : ``integer``
        Positive index for spherical coordinate.

    See Also
    --------
    :func:`.spherical_transform`
    :class:`nengo.dists.SqrtBeta`

    References
    ----------
    .. [#] K.-T. Fang and Y. Wang, Number-Theoretic Methods in Statistics.
       Chapman & Hall, 1994.

    Examples
    --------
    >>> from nengolib.stats import SphericalCoords
    >>> coords = SphericalCoords(3)

    >>> import matplotlib.pyplot as plt
    >>> x = np.linspace(0, 1, 1000)
    >>> plt.figure(figsize=(8, 8))
    >>> plt.subplot(411)
    >>> plt.title(str(coords))
    >>> plt.ylabel("Samples")
    >>> plt.hist(coords.sample(1000), bins=50, normed=True)
    >>> plt.subplot(412)
    >>> plt.ylabel("PDF")
    >>> plt.plot(x, coords.pdf(x))
    >>> plt.subplot(413)
    >>> plt.ylabel("CDF")
    >>> plt.plot(x, coords.cdf(x))
    >>> plt.subplot(414)
    >>> plt.ylabel("PPF")
    >>> plt.plot(x, coords.ppf(x))
    >>> plt.xlabel("x")
    >>> plt.show()
    """

    def __init__(self, m):
        super(SphericalCoords, self).__init__()
        self.m = m

    def __repr__(self):
        return "%s(%r)" % (type(self).__name__, self.m)

    def sample(self, n, d=None, rng=np.random):
        """Samples ``n`` points in ``d`` dimensions."""
        shape = self._sample_shape(n, d)
        y = rng.uniform(size=shape)
        return self.ppf(y)

    def pdf(self, x):
        """Evaluates the PDF along the values ``x``."""
        return np.pi * np.sin(np.pi * x) ** (self.m - 1) / beta(self.m / 2.0, 0.5)

    def cdf(self, x):
        """Evaluates the CDF along the values ``x``."""
        y = 0.5 * betainc(self.m / 2.0, 0.5, np.sin(np.pi * x) ** 2)
        return np.where(x < 0.5, y, 1 - y)

    def ppf(self, y):
        """Evaluates the inverse CDF along the values ``x``."""
        y_reflect = np.where(y < 0.5, y, 1 - y)
        z_sq = betaincinv(self.m / 2.0, 0.5, 2 * y_reflect)
        x = np.arcsin(np.sqrt(z_sq)) / np.pi
        return np.where(y < 0.5, x, 1 - x)


def _rd_generate(n, d, seed=0.5):
    """Generates the first ``n`` points in the ``R_d`` sequence."""

    # http://extremelearning.com.au/unreasonable-effectiveness-of-quasirandom-sequences/
    def gamma(d, n_iter=20):
        """Newton-Raphson-Method to calculate g = phi_d."""
        x = 1.0
        for _ in range(n_iter):
            x -= (x ** (d + 1) - x - 1) / ((d + 1) * x ** d - 1)
        return x

    g = gamma(d)
    alpha = np.zeros(d)
    for j in range(d):
        alpha[j] = (1 / g) ** (j + 1) % 1

    z = np.zeros((n, d))
    z[0] = (seed + alpha) % 1
    for i in range(1, n):
        z[i] = (z[i - 1] + alpha) % 1

    return z


class Rd(Distribution):
    """Rd sequence for quasi Monte Carlo sampling the ``[0, 1]``--cube.

    This is similar to ``np.random.uniform(0, 1, size=(num, d))``, but with
    the additional property that each ``d``--dimensional point is `uniformly
    scattered`.

    This is based on the tutorial and code from [#]_. For `d=2` this is often
    called the Padovan sequence. [#]_

    See Also
    --------
    :class:`.Sobol`
    :class:`.ScatteredCube`
    :func:`.spherical_transform`
    :class:`.ScatteredHypersphere`

    References
    ----------
    .. [#] http://extremelearning.com.au/unreasonable-effectiveness-of-quasirandom-sequences/
    .. [#] http://oeis.org/A000931

    Examples
    --------
    >>> from nengolib.stats import Rd
    >>> rd = Rd().sample(10000, 2)

    >>> import matplotlib.pyplot as plt
    >>> plt.figure(figsize=(6, 6))
    >>> plt.scatter(*rd.T, c=np.arange(len(rd)), cmap='Blues', s=7)
    >>> plt.show()
    """  # noqa: E501

    def __repr__(self):
        return "%s()" % (type(self).__name__)

    def sample(self, n, d=1, rng=np.random):
        """Samples ``n`` points in ``d`` dimensions."""
        if d == 1:
            # Tile the points optimally. TODO: refactor
            return np.linspace(1.0 / n, 1, n)[:, None]
        if d is None or not isinstance(d, (int, np.integer)) or d < 1:
            # TODO: this should be raised when the ensemble is created
            raise ValueError("d (%d) must be positive integer" % d)
        return _rd_generate(n, d)


class ScatteredCube(Distribution):
    """Number-theoretic distribution over the hypercube.

    Transforms quasi Monte Carlo samples from the unit hypercube
    to range between ``low`` and ``high``. These bounds may optionally be
    ``array_like`` with shape matching the sample dimensionality.

    Parameters
    ----------
    low : ``float`` or ``array_like``, optional
        Lower-bound(s) for each sample. Defaults to ``-1``.
    high : ``float`` or ``array_like``, optional
        Upper-bound(s) for each sample. Defaults to ``+1``.

    Other Parameters
    ----------------
    base : :class:`nengo.dists.Distribution`, optional
        The base distribution from which to draw `quasi Monte Carlo` samples.
        Defaults to :class:`.Rd` and should not be changed unless
        you have some alternative `number-theoretic sequence` over ``[0, 1]``.

    See Also
    --------
    :attr:`.cube`
    :class:`.Rd`
    :class:`.Sobol`
    :class:`.ScatteredHypersphere`

    Notes
    -----
    The :class:`.Rd` and :class:`.Sobol` distributions are deterministic.
    Nondeterminism comes from a random ``d``--dimensional shift (with
    wrap-around).

    Examples
    --------
    >>> from nengolib.stats import ScatteredCube
    >>> s1 = ScatteredCube([-1, -1, -1], [1, 1, 0]).sample(1000, 3)
    >>> s2 = ScatteredCube(0, 1).sample(1000, 3)
    >>> s3 = ScatteredCube([-1, .5, 0], [-.5, 1, .5]).sample(1000, 3)

    >>> import matplotlib.pyplot as plt
    >>> from mpl_toolkits.mplot3d import Axes3D
    >>> plt.figure(figsize=(6, 6))
    >>> ax = plt.subplot(111, projection='3d')
    >>> ax.scatter(*s1.T)
    >>> ax.scatter(*s2.T)
    >>> ax.scatter(*s3.T)
    >>> plt.show()
    """

    def __init__(self, low=-1, high=+1, base=Rd()):
        super(ScatteredCube, self).__init__()
        self.low = np.atleast_1d(low)
        self.high = np.atleast_1d(high)
        self.w = self.high - self.low
        self.base = base

    def __repr__(self):
        return "%s(low=%r, high=%r, base=%r)" % (
            type(self).__name__,
            self.low,
            self.high,
            self.base,
        )

    def sample(self, n, d=1, rng=np.random):
        """Samples ``n`` points in ``d`` dimensions."""
        u = self.base.sample(n, d, rng)

        # shift everything by the same random constant (with wrap-around)
        u = (u + rng.uniform(size=d)[None, :]) % 1.0

        return u * self.w[None, :] + self.low[None, :]


class ScatteredHypersphere(UniformHypersphere):
    """Number--theoretic distribution over the hypersphere and hyperball.

    Applies the :func:`.spherical_transform` to the number-theoretic
    sequence :class:`.Rd` to obtain uniformly scattered samples.

    This distribution has the nice mathematical property that the
    `discrepancy` between the `empirical distribution` and :math:`n` samples
    is :math:`\\widetilde{\\mathcal{O}}\\left(\\frac{1}{n}\\right)` as opposed
    to :math:`\\mathcal{O}\\left(\\frac{1}{\\sqrt{n}}\\right)` for the `Monte
    Carlo` method. [#]_ This means that the number of samples are effectively
    squared, making this useful as a means for sampling ``eval_points`` and
    ``encoders`` in Nengo.

    See :doc:`notebooks/research/sampling_high_dimensional_vectors` for
    mathematical details.

    Parameters
    ----------
    surface : ``boolean``
        Set to ``True`` to restrict the points to the surface of the ball
        (i.e., the sphere, with one lower dimension). Set to ``False`` to
        sample from the ball. See also :attr:`.sphere` and :attr:`.ball` for
        pre-instantiated objects with these two options respectively.

    Other Parameters
    ----------------
    base : :class:`nengo.dists.Distribution`, optional
        The base distribution from which to draw `quasi Monte Carlo` samples.
        Defaults to :class:`.Rd` and should not be changed unless
        you have some alternative `number-theoretic sequence` over ``[0, 1]``.

    See Also
    --------
    :attr:`.sphere`
    :attr:`.ball`
    :class:`nengo.dists.UniformHypersphere`
    :class:`.Rd`
    :class:`.Sobol`
    :func:`.spherical_transform`
    :class:`.ScatteredCube`

    Notes
    -----
    The :class:`.Rd` and :class:`.Sobol` distributions are deterministic.
    Nondeterminism comes from a random ``d``--dimensional rotation
    (see :func:`.random_orthogonal`).

    The nengolib logo was created using this class with the Sobol sequence.

    References
    ----------
    .. [#] K.-T. Fang and Y. Wang, Number-Theoretic Methods in Statistics.
       Chapman & Hall, 1994.

    Examples
    --------
    >>> from nengolib.stats import ball, sphere
    >>> b = ball.sample(1000, 2)
    >>> s = sphere.sample(1000, 3)

    >>> import matplotlib.pyplot as plt
    >>> from mpl_toolkits.mplot3d import Axes3D
    >>> plt.figure(figsize=(6, 3))
    >>> plt.subplot(121)
    >>> plt.title("Ball")
    >>> plt.scatter(*b.T, s=10, alpha=.5)
    >>> ax = plt.subplot(122, projection='3d')
    >>> ax.set_title("Sphere").set_y(1.)
    >>> ax.patch.set_facecolor('white')
    >>> ax.set_xlim3d(-1, 1)
    >>> ax.set_ylim3d(-1, 1)
    >>> ax.set_zlim3d(-1, 1)
    >>> ax.scatter(*s.T, s=10, alpha=.5)
    >>> plt.show()
    """

    def __init__(self, surface, base=Rd()):
        super(ScatteredHypersphere, self).__init__(surface)
        self.base = base

    def __repr__(self):
        return "%s(surface=%r, base=%r)" % (
            type(self).__name__,
            self.surface,
            self.base,
        )

    def sample(self, n, d=1, rng=np.random):
        """Samples ``n`` points in ``d`` dimensions."""
        if d == 1:
            return super(ScatteredHypersphere, self).sample(n, d, rng)

        if self.surface:
            samples = self.base.sample(n, d - 1, rng)
            radius = 1.0
        else:
            samples = self.base.sample(n, d, rng)
            samples, radius = samples[:, :-1], samples[:, -1:] ** (1.0 / d)

        mapped = spherical_transform(samples)

        # radius adjustment for ball versus sphere, and a random rotation
        rotation = random_orthogonal(d, rng=rng)
        return np.dot(mapped * radius, rotation)


cube = ScatteredCube()

sphere = ScatteredHypersphere(surface=True)

ball = ScatteredHypersphere(surface=False)
