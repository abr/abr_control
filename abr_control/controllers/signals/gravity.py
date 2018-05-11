class Gravity(Signal):
    """
    functions for either bootstrapping learning or for artificial perturbations

    """
    def __init__(self, robot_config):
        self.robot_config = robot_config

    def generate(self, x):
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
