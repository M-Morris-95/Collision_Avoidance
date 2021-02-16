import numpy as np
import copy
from scipy.interpolate import splev, splrep
import matplotlib.pyplot as plt

def smooth(route):
    scale = 5
    loop_min_size = 1.0

    hist = np.asarray(route)
    hist = (hist * scale).round()
    indexes = np.unique(hist, axis=0, return_index=True)[1]
    hist = np.asarray([route[index] for index in sorted(indexes)])

    ## remove loops
    l_map = np.zeros((hist.shape[0], hist.shape[0])) + 1
    for i in range(hist.shape[0]):
        for j in range(i + 1, hist.shape[0]):
            l_map[i, j] = np.sqrt(np.sum(np.square(hist[i] - hist[j])))
    positions = np.asarray(np.where(l_map < loop_min_size/scale)).T

    temp = []
    for i in range(len(positions) - 1):
        if positions[i, 0] != positions[i + 1, 0]:
            if positions[i, 1] - positions[i, 0] > 1:
                temp.append(positions[i])
    temp = np.asarray(temp)

    c_max = 0
    n_temp = []
    for i in range(len(temp)):
        if temp[i, 1] > c_max:
            n_temp.append(temp[i])
            c_max = temp[i, 1]
    n_temp = np.asarray(n_temp)

    c_max = -1
    n2_temp = []
    for i in range(len(n_temp)):
        if n_temp[i, 0] > c_max:
            n2_temp.append(n_temp[i])
            c_max = n_temp[i, 1]
    n2_temp = np.asarray(n2_temp)

    if n2_temp.size == 0:
        new_route = hist
    elif n2_temp.size == 2:
        new_route = np.concatenate([hist[:n2_temp[0, 0]], hist[n2_temp[0, 1]:]])

    else:
        new_route = [hist[0:n2_temp[0, 0]]]
        for i in range(len(n2_temp) - 1):
            new_route.append(hist[n2_temp[i, 1]:n2_temp[i + 1, 0]])
            new_route.append(hist[n2_temp[i + 1, 1]:])


        for i in range(len(new_route) - 1, -1, -1):
            if new_route[i].size == 0:
                new_route.pop(i)
        new_route = np.concatenate(new_route)


    return new_route

class path_finder():
    def __init__(self, start = np.array([1,5]), end = np.array([9,5]), sigma = 0.5, speed=2):
        self.pos_hist = [start]

        self.pos = start
        self.end = end
        self.dxdy = np.sqrt(np.square(start - end).sum())

        self.tol = 0.5
        self.mass = 1.0
        self.f_goal = 10

        self.noise = np.asarray([0,0])
        self.noise_param = {'mu':0, 'sigma':sigma, 'decay_period':100}

        self.pos_hist = [start]
        self.vel = np.array([0,0])
        self.vel_hist = [self.vel]

        self.speed = speed

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

    def get_route(self, map, dt, pos, vel=np.asarray([0,0])):
        noise_array = np.linspace(0.1, 1.5, 15)

        for sigma in noise_array:
            for _ in range(5):
                self.noise_param['sigma'] = sigma
                self.pos = copy.copy(pos)
                self.vel = copy.copy(vel)
                self.pos_hist = [self.pos]
                self.vel_hist = [self.vel]


                for i in range(int((50) / dt)):
                    self.next_pos(map, dt=dt)

                    if self.terminal():
                        break
                if self.terminal():
                    break
            if self.terminal():
                break

        route = smooth(self.pos_hist)
        t = np.linspace(0, (route.shape[0]-1) * dt, route.shape[0])

        try:
            route_len = np.sum(np.sqrt(np.sum((np.square(np.diff(route, axis=0))), 1)))

            max_t = route_len / self.speed

            new_times = np.linspace(0, (route.shape[0] - 1) * dt, int(max_t / dt))
            splx = splrep(t, route[:, 0])
            sply = splrep(t, route[:, 1])

            splev(new_times, splx)
            splev(new_times, sply)
            route = np.asarray([splev(new_times, splx),splev(new_times, sply)]).T
        except:
            pass
        return route




    def re_init(self, pos, vel):
        self.pos = pos
        self.vel = vel
        self.pos_hist = [pos]
        self.vel_hist = [vel]


def plr(route, k=0):
    plt.scatter(route[k:, 0], route[k:, 1])
    plt.xlim([0, 10])
    plt.ylim([0, 10])
    plt.show()
