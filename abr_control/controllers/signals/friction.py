class Friction(Signal):
    """
    functions for either bootstrapping learning or for artificial perturbations

    """
    def __init__(self, robot_config, Fn=4, uk=0.42, us=0.74, vs=0.1, Fv=1.2,
        self.robot_config = robot_config
        self.Fn = Fn
        self.uk = uk
        self.us = us
        self.vs = vs
        self.Fv = Fv

    @property
    def params(self):
        params = {'source': 'friction',
                  'Fn': self.Fn,
                  'uk': self.uk,
                  'us': self.us,
                  'vs': self.vs,
                  'Fv': self.Fv}

    def generate(self, dq, show_plot=False):
        # using static and kinetic coefficients of friction for steel on steel

        # if len(dq) > self.adapt_dim:
        #     # if entire adaptive input is passed in, only take the velocity
        #     # (second half)
        #     v = np.copy(dq[self.adapt_dim:])
        # else:
        #     # if dq matches the length of adaptive dim then we have received
        #     # velocity directly
        #     v = np.copy(dq)
        sgn_v = np.sign(dq)
        # forces selected to have friction in a reasonable force range wrt the
        # forces the arm moves with (friction in range of 2-4N for typical
        # movement)
        Fc = -self.uk * self.Fn * sgn_v
        Fs = -self.us * self.Fn * sgn_v
        Ff = Fc + (Fs-Fc) * np.exp(-sgn_v * dq/self.vs) - self.Fv * dq

        # TODO: will this check affect loop speed at all? Worth having?
        if show_plot:
            import matplotlib
            matplotlib.use("TKAgg")
            from matplotlib import pyplot as plt

            plt.figure()
            plt.title('Friction')
            plt.ylabel('Force [Nm]')
            plt.xlabel('Velocity [rad/sec]')
            plt.plot(Ff)
            plt.legend()
            plt.show()

        return(Ff)
