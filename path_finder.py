import numpy as np
import copy
from scipy.interpolate import splev, splrep
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return np.asarray([x, y])

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
        self.max_speed = 1
        self.noise = np.asarray([0,0])
        self.noise_param = {'mu':0, 'sigma':sigma, 'decay_period':100}

        self.pos_hist = [start]
        self.vel = np.array([0,0])
        self.vel_hist = [self.vel]

        self.speed = speed

    def ornstein_uhlenbeck_noise(self):
        self.noise = self.noise * (1-1/self.noise_param['decay_period']) + np.random.normal(self.noise_param['mu'], self.noise_param['sigma'], 2)

    def get_f(self, map, pos=None):
        if pos is None:
            fx, fy = map.get_f(self.pos)
            self.dxdy = self.end - self.pos
        else:
            map.get_f(pos)
            self.dxdy = self.end - [ps]


        theta = np.arctan2(self.dxdy[1], self.dxdy[0])

        fx_goal = self.f_goal * np.cos(theta)
        fy_goal = self.f_goal * np.sin(theta)

        fx = fx + fx_goal
        fy = fy + fy_goal

        self.ornstein_uhlenbeck_noise()

        return np.asarray([fx, fy]) + self.noise



    def next_pos(self, map, force=None, dt = 0.05):
        self.map = map
        if force is None:
            force = self.get_f(map)




        self.vel = self.vel + (force / self.mass) * dt


        scale = self.max_speed/np.sqrt(np.square(self.vel).sum())
        self.vel = self.vel * scale
        self.pos = self.pos + self.vel * dt

        self.vel_hist.append(self.vel)
        self.pos_hist.append(self.pos)

    def terminal(self, pos = None, end = None):
        if end is None:
            end = self.end
        if pos is None:
            pos = self.pos

        dist = np.sqrt(np.square(pos - end).sum())

        if dist < self.tol:
            return True
        return False

    def get_route_ode_int(self, map, pos, vel=np.asarray([0,0]), dt=0.05):
        def dy_dx(U, _):
            U = U.reshape((2, 2))
            if self.terminal(pos=U[0], end=self.finish):
                self.flag=True
                return np.asarray([0, 0, 0, 0])

            dxdy = self.end - U[0]
            theta = np.arctan2(dxdy[1], dxdy[0])
            f_goal = np.asarray([10 * np.cos(theta), 10 * np.sin(theta)])
            f_map = map.get_f(U[0])
            f_tot = f_map + f_goal
            acc = f_tot / self.mass

            scale = self.max_speed / np.sqrt(np.square(U[1]).sum())
            vel = U[1] * min(1, scale)
            # vel = U[1]

            return np.asarray([vel, acc]).flatten()

        num_thetas = 8
        dist = 4.0
        goals = np.tile(self.end.T, [num_thetas, 1]) + pol2cart(rho=dist, phi=np.linspace(0, np.pi * 2, num_thetas)).T
        goals = np.concatenate([goals, self.end[np.newaxis,:]])
        goals = np.minimum(goals, np.asarray([9, 9]))
        goals = np.maximum(goals, np.asarray([1, 1]))



        positions = []
        velocities = []
        lengths = []

        for goal in goals:
            self.finish = goal
            self.flag = False
            U0 = np.asarray([pos, [0.1,0.1]]).flatten()
            t = np.linspace(0, 20, 100)
            Us = odeint(dy_dx, U0, t)

            if self.flag:
                self.flag=False
                Us= Us[Us != Us[-1]].reshape((-1, 4))
                self.finish = self.end
                Us2 = odeint(dy_dx, Us[-1], t)
                if self.flag:
                    Us2 = Us2[Us2 != Us2[-1]].reshape((-1, 4))

                Us = np.concatenate([Us, Us2])

            positions.append(Us[:,:2])
            velocities.append(Us[:,2:])
            lengths.append(np.asarray(np.sum(np.sqrt(np.sum((np.square(np.diff(Us[:,:2], axis=0))), 1)))))

            plt.plot(Us[:, 0], Us[:, 1])
        np.asarray(lengths).argmin()

        route = positions[np.asarray(lengths).argmin()]
        plt.plot(route[:, 0], route[:, 1], color='red')
        plt.xlim([0,10])
        plt.ylim([0,10])
        plt.show()
        return route


    def get_route(self, map, pos, vel=np.asarray([0,0]), dt=0.05):

        num_thetas = 8
        dist = 4.0

        goals = np.tile(self.end.T, [num_thetas, 1]) + pol2cart(rho=dist, phi=np.linspace(0, np.pi * 2, num_thetas)).T
        goals = np.concatenate([goals, self.end[np.newaxis,:]])

        goals = np.minimum(goals, 9)
        goals = np.maximum(goals, 1)

        positions = []
        velocities = []
        lengths = []
        terminal=[]
        max_len = 100

        for goal in goals:
            self.pos = copy.copy(pos)
            self.vel = copy.copy(vel)
            self.pos_hist = [self.pos]
            self.vel_hist = [self.vel]
            flag1 = True
            len = 0
            for i in range(int((30) / dt)):
                if flag1:
                    dxdy = goal - self.pos
                else:
                    dxdy = self.end - self.pos

                theta = np.arctan2(dxdy[1], dxdy[0])

                F = map.get_f(self.pos) + np.asarray([self.f_goal * np.cos(theta), self.f_goal * np.sin(theta)])

                self.next_pos(map, force=F, dt=dt)
                len = len + np.square(self.pos_hist[-1] - self.pos_hist[-2]).sum()
                if self.terminal(end=goal):
                    flag1 = False
                if self.terminal():
                    max_len = len
                    break
                if len > max_len:
                    break
            # route = smooth(np.asarray(self.pos_hist))
            # route = self.resample(np.asarray(self.pos_hist), dt=dt)
            # positions.append(route)

            positions.append(np.asarray(self.pos_hist))

            velocities.append(np.asarray(self.vel_hist))
            terminal.append(self.terminal())
            lengths.append(np.asarray(np.sum(np.sqrt(np.sum((np.square(np.diff(self.pos_hist, axis=0))), 1)))))

        lengths = np.asarray(lengths) * np.asarray(terminal) * -1
        route = positions[lengths.argmin()]
        return route, positions

    def resample(self, route, dt=0.005):
        try:
            route_len = np.sum(np.sqrt(np.sum((np.square(np.diff(route, axis=0))), 1)))

            num_time_steps = int(route_len / self.speed / dt)
            end_time = (route.shape[0] - 1) * dt,

            crude_t = np.linspace(0, end_time, 50)
            t = np.linspace(0, (route.shape[0]-1) * dt, route.shape[0])

            splx = splrep(t, route[:, 0])
            sply = splrep(t, route[:, 1])

            x = splev(crude_t, splx)
            y = splev(crude_t, sply)

            fine_t = np.linspace(0, end_time, num_time_steps)
            splx = splrep(crude_t, x)
            sply = splrep(crude_t, y)

            x = splev(fine_t, splx)
            y = splev(fine_t, sply)

            route = np.asarray([x, y]).T.squeeze()
        except:
            pass
        return  route

    def get_route_noise(self, map, pos, vel=np.asarray([0,0]), dt=None):
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

            num_time_steps = int(route_len / self.speed / dt)
            end_time = (route.shape[0] - 1) * dt,

            crude_t = np.linspace(0, end_time, 50)
            splx = splrep(t, route[:, 0])
            sply = splrep(t, route[:, 1])

            x = splev(crude_t, splx)
            y = splev(crude_t, sply)

            fine_t = np.linspace(0, end_time, num_time_steps)
            splx = splrep(crude_t, x)
            sply = splrep(crude_t, y)

            x = splev(fine_t, splx)
            y = splev(fine_t, sply)

            route = np.asarray([x,y]).T.squeeze()
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
