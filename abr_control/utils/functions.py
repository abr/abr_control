class Functions():
    """
    functions for either bootstrapping learning or for artificial perturbations

    """
    def __init__(self, robot_config):
        self.robot_config = robot_config

    def gravity(self, x):
        #TODO: need to generalize this for number of joints

        #q1, q2 = x[:self.adapt_dim]  # x = [q, dq]
        q0, q1, q2 = x[:self.adapt_dim]  # x = [q, dq]
        g_avg = []
        dist = nengo.dists.UniformHypersphere()
        samples = dist.sample(1000, d=self.robot_config.N_JOINTS - self.adapt_dim)
        for sample in samples:
            #q = np.hstack([sample[0], q0, q1, q2, sample[1:]])
            q = np.hstack([q0, q1, q2, sample])
            q = self.robot_config.scaleup('q', q)
            pars = tuple(q) + tuple([0, 0, 0])
            g = np.dot(self.JEE(*pars).T, self.fake_gravity)
            g_avg.append(g.squeeze())
        #g_avg = np.mean(np.array(g_avg), axis=0)[[1, 2]]
        g_avg = np.mean(np.array(g_avg), axis=0)[:3]
        return -g_avg*self.decimal_scale

    def friction(self, x, Fn=4, uk=0.42, us=0.74, vs=0.1, Fv=1.2,
            show_plot=False):
        # using static and kinetic coefficients of friction for steel on steel

        # if len(x) > self.adapt_dim:
        #     # if entire adaptive input is passed in, only take the velocity
        #     # (second half)
        #     v = np.copy(x[self.adapt_dim:])
        # else:
        #     # if x matches the length of adaptive dim then we have received
        #     # velocity directly
        #     v = np.copy(x)
        sgn_v = np.sign(x)
        # forces selected to have friction in a reasonable force range wrt the
        # forces the arm moves with (friction in range of 2-4N for typical
        # movement)
        Fc = -uk * Fn * sgn_v
        Fs = -us * Fn * sgn_v
        Ff = Fc + (Fs-Fc) * np.exp(-sgn_v * v/vs) - Fv * v

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


