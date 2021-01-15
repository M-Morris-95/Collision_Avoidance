import numpy as np
import matplotlib
import time
import copy
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator
import numpy as np

class map_2d():
    def __init__(self, size = [50,50], xlim = [0,10], ylim = [0,10], start = np.asarray([1,5]), end = np.asarray([9,5])):
        self.xlim = xlim
        self.ylim = ylim

        self.start = start
        self.end = end

        self.size_x = size[0]
        self.size_y = size[1]

        self.range_x = xlim[1] - xlim[0]
        self.range_y = ylim[1] - ylim[0]

        self.dy = (ylim[1] - ylim[0]) / size[1]
        self.dx = (xlim[1] - xlim[0]) / size[0]

        self.d_max = 2.5 # limit of influence from map
        self.d_min = min(self.dx, self.dy) # Stops F = inf

        # generate 2 2d grids for the x & y bounds
        self.y, self.x = np.meshgrid(np.arange(xlim[0], xlim[1]+self.dx, self.dx),
                                     np.arange(ylim[0], xlim[1]+self.dy, self.dy))

        self.z = np.zeros(self.x.shape)

        self.z[0, :] = 1
        self.z[-1, :] = 1
        self.z[:, 0] = 1
        self.z[:, -1] = 1

        return None

    def fill_map(self):
        n_shapes = 50
        size_of_shapes = 5

        for _ in range(n_shapes):
            pos = [(self.size_x*np.random.random(1)).astype(int),
                   (self.size_y*np.random.random(1)).astype(int)]

            size = (size_of_shapes*np.random.random(1)).astype(int)[0]

            self.z[int(pos[0]-size/2) : int(pos[0]+size/2), int(pos[1]-size/2) : int(pos[1]+size/2)] = 1

        size_start_end = 10
        self.z[int(self.end[0] - size_start_end / 2): int(self.end[0] + size_start_end / 2),
        int(self.end[1] - size_start_end / 2): int(self.end[1] + size_start_end / 2)] = 0

        self.z[int(self.start[0] - size_start_end / 2): int(self.start[0] + size_start_end / 2),
        int(self.start[1] - size_start_end / 2): int(self.start[1] + size_start_end / 2)] = 0

    def plot(self, to_plot, title = None):
        cmap = plt.get_cmap('PiYG')

        levels = MaxNLocator(nbins=15).tick_values(self.z.min(), self.z.max())
        norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)

        fig, ax0, = plt.subplots(1, 1)

        # contours are *point* based plots, so convert our bound into point
        # centers

        cf = ax0.contourf(self.x,
                          self.y,
                          to_plot,
                          antialiased = False)


        fig.colorbar(cf, ax=ax0)
        ax0.set_title(title)

        # adjust spacing between subplots so `ax1` title and `ax0` tick labels
        # don't overlap
        fig.tight_layout()

        plt.show()

    def get_f(self, pos, type = 'square'):
        dx = pos[0] - self.x
        dy = pos[1] - self.y

        dxdy2 = np.square(dx) + np.square(dy)
        dxdy2[dxdy2 > self.d_max ** 2] = np.inf
        dxdy2[dxdy2 < self.d_min ** 2] = self.d_min ** 2

        if type == 'square':
            self.f_map = np.divide(0.4*self.z, dxdy2)

        if type == 'gaussian':
            self.f_map = 5*np.exp(-np.sqrt(dxdy2)) * self.z


        thetas = np.arctan2(dx, dy)

        fx = np.multiply(np.sin(thetas), self.f_map).sum()
        fy = np.multiply(np.cos(thetas), self.f_map).sum()

        return fx, fy, self.f_map.sum()

class pos_solv():
    def __init__(self, start = np.array([1,5]), end = np.array([9,5])):
        self.pos_hist = [start]

        self.pos = start
        self.end = end
        self.dxdy = np.sqrt(np.square(start - end).sum())

        self.tol = 0.25
        self.mass = 0.0001
        self.f_goal = 10

        self.noise = np.asarray([0,0])
        self.noise_param = {'mu':0, 'sigma':0.5, 'decay_period':100}

        self.vel = np.array([0,0])
        self.vel_hist = [self.vel]

    def ornstein_uhlenbeck_noise(self):
        self.noise = self.noise * (1-1/self.noise_param['decay_period']) + np.random.normal(self.noise_param['mu'], self.noise_param['sigma'], 2)

    def get_f(self, map):
        fx, fy, _ = map.get_f(self.pos)

        self.dxdy = self.end - self.pos

        theta = np.arctan2(self.dxdy[1], self.dxdy[0])

        fx_goal = self.f_goal * np.cos(theta)
        fy_goal = self.f_goal * np.sin(theta)



        fx = fx + fx_goal
        fy = fy + fy_goal

        self.ornstein_uhlenbeck_noise()

        return np.asarray([fx, fy]) + self.noise

    def next_pos(self, map, dt = 0.005):
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


start = np.asarray([1,1])
end = np.asarray([9,9])


map = map_2d(start=start, end=end)
map.fill_map()


for run in range(20):
    pos_ode = pos_solv(start=start, end=end)

    pos_ode.noise_param['sigma'] = run * 0.1

    for i in range(10000):
        pos_ode.next_pos(map)
        if pos_ode.terminal():
            break

    pos = (np.asarray(pos_ode.pos_hist) * (1 / map.dx)).astype(int)
    path_map = copy.copy(map.z)
    for i in range(pos.shape[0]):
        path_map[pos[i][0], pos[i][1]] = -1

    map.plot(path_map)

    if pos_ode.terminal():
        break

f_all = np.zeros(map.x.shape)
fx_all = np.zeros(map.x.shape)
fy_all = np.zeros(map.x.shape)

for idx_x, x in enumerate(np.arange(map.xlim[0], map.xlim[1] + map.dx, map.dx)):
    for idx_y, y in enumerate(np.arange(map.ylim[0], map.xlim[1] + map.dy, map.dy)):
        fx, fy, f = map.get_f([x,y])

        dxdy = pos_ode.end - np.asarray([x,y])

        theta = np.arctan2(dxdy[1], dxdy[0])

        fx_goal = pos_ode.f_goal * np.cos(theta)
        fy_goal = pos_ode.f_goal * np.sin(theta)

        f_all[idx_x, idx_y] = f
        fx_all[idx_x, idx_y] = fx + fx_goal
        fy_all[idx_x, idx_y] = fy + fy_goal

map.plot(map.z)
map.plot(f_all)

fig, ax = plt.subplots()
q = ax.quiver(map.x, map.y, fx_all, fy_all)
ax.quiverkey(q, X=0.3, Y=1.1, U=10,
             label='Quiver key, length = 10', labelpos='E')
plt.show()