import numpy as np

class pos_solv():
    def __init__(self, start = np.array([1,5]), end = np.array([9,5]), sigma = 0.5):
        self.pos_hist = [start]

        self.pos = start
        self.end = end
        self.dxdy = np.sqrt(np.square(start - end).sum())

        self.tol = 0.5
        self.mass = 1.0
        self.f_goal = 10

        self.noise = np.asarray([0,0])
        self.noise_param = {'mu':0, 'sigma':sigma, 'decay_period':100}

        self.vel = np.array([0,0])
        self.vel_hist = [self.vel]

    def ornstein_uhlenbeck_noise(self):
        self.noise = self.noise * (1-1/self.noise_param['decay_period']) + np.random.normal(self.noise_param['mu'], self.noise_param['sigma'], 2)

    def get_f(self, map):
        fx, fy = map.get_f(self.pos)

        self.dxdy = self.end - self.pos

        theta = np.arctan2(self.dxdy[1], self.dxdy[0])

        fx_goal = self.f_goal * np.cos(theta)
        fy_goal = self.f_goal * np.sin(theta)

        fx = fx + fx_goal
        fy = fy + fy_goal

        self.ornstein_uhlenbeck_noise()

        return np.asarray([fx, fy]) + self.noise

    def next_pos(self, map, dt = 0.05):
        force = self.get_f(map)

        self.vel = self.vel + (force / self.mass) * dt
        self.vel = np.maximum([-1, -1], self.vel)
        self.vel = np.minimum([1, 1], self.vel)

        self.pos = self.pos + self.vel * dt

        self.vel_hist.append(self.vel)
        self.pos_hist.append(self.pos)

    def terminal(self):
        dist = np.sqrt(np.square(self.pos - self.end).sum())

        if dist < self.tol:
            return True
        return False

    def update(self, pos=0,vel=0):
        self.pos = pos
        self.vel = vel

        self.vel_hist.append(self.vel)
        self.pos_hist.append(self.pos)
