import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator
import numpy as np

class map_2d():
    def __init__(self, size = [80,80], xlim = [0,10], ylim = [0,10]):
        self.xlim = xlim
        self.ylim = ylim

        self.size_x = size[0]
        self.size_y = size[0]


        self.range_x = xlim[1] - xlim[0]
        self.range_y = ylim[1] - ylim[0]

        self.dy = (ylim[1] - ylim[0]) / size[1]
        self.dx = (xlim[1] - xlim[0]) / size[0]

        self.d_min = min(self.dx, self.dy)

        # generate 2 2d grids for the x & y bounds
        self.y, self.x = np.meshgrid(np.arange(xlim[0], xlim[1]+self.dx, self.dx),
                                     np.arange(ylim[0], xlim[1]+self.dy, self.dy))

        self.z = np.zeros(self.x.shape)

        return None

    def fill_map(self):
        n_shapes = 10
        size_of_shapes = 10

        for _ in range(n_shapes):
            pos = [(self.size_x*np.random.random(1)).astype(int),
                   (self.size_y*np.random.random(1)).astype(int)]

            size = (size_of_shapes*np.random.random(1)).astype(int)[0]

            self.z[int(pos[0]-size/2) : int(pos[0]+size/2), int(pos[1]-size/2) : int(pos[1]+size/2)] = 1

    def plot(self, to_plot, title = None):
        cmap = plt.get_cmap('PiYG')

        levels = MaxNLocator(nbins=15).tick_values(self.z.min(), self.z.max())
        norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)

        fig, ax0, = plt.subplots(1, 1)

        # contours are *point* based plots, so convert our bound into point
        # centers

        cf = ax0.contourf(self.x,
                          self.y, to_plot)


        fig.colorbar(cf, ax=ax0)
        ax0.set_title(title)

        # adjust spacing between subplots so `ax1` title and `ax0` tick labels
        # don't overlap
        fig.tight_layout()

        plt.show()

    def get_f(self, pos):
        dx = pos[0] - self.x
        dy = pos[1] - self.y

        self.dxdy = np.maximum(np.square(dx) + np.square(dy) , self.d_min)

        thetas = np.arctan2(dx, dy)

        self.f_map = np.divide(self.z, self.dxdy)

        fx = np.multiply(np.sin(thetas), self.f_map).sum()
        fy = np.multiply(np.cos(thetas), self.f_map).sum()

        return fx, fy, self.f_map.sum()

class pos_solv():
    def __init__(self, start = np.array([5,0]), end = np.array([5,10])):
        self.pos_hist = [start]

        self.pos = start
        self.end = end

        self.dxdy = np.sqrt(np.square(start - end).sum())
        self.tol = 1

        self.mass = 20

        self.f_goal = 20
        self.vel = np.array([0,0])
        self.vel_hist = [self.vel]

    def get_f(self, map):
        fx, fy, _ = map.get_f(self.pos)

        self.dxdy = self.end - self.pos

        theta = np.arctan2(self.dxdy[1], self.dxdy[0])

        fx_goal = self.f_goal * np.cos(theta)
        fy_goal = self.f_goal * np.sin(theta)

        fx = fx + fx_goal
        fy = fy + fy_goal

        return np.asarray([fx, fy])

    def next_pos(self, map, dt = 0.005):
        force = self.get_f(map)

        self.vel = self.vel + force / self.mass * dt
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

map = map_2d()
pos_ode = pos_solv()
map.fill_map()
map.plot(map.z, 'map')

for i in range(10000):
    pos_ode.next_pos(map)
    if pos_ode.terminal():
        break

plt.subplot(2,1,1)
plt.title('position')
plt.plot(np.asarray(pos_ode.pos_hist)[:,0], np.asarray(pos_ode.pos_hist)[:,1])
plt.xlim([0,10])
plt.ylim([0,10])
plt.subplot(2,1,2)
plt.title('velocity')
plt.plot(np.asarray(pos_ode.vel_hist)[:,0], np.asarray(pos_ode.vel_hist)[:,1])
plt.xlim([-10,10])
plt.ylim([-10,10])
plt.show()





# f_all = np.zeros(map.x.shape)
# fx_all = np.zeros(map.x.shape)
# fy_all = np.zeros(map.x.shape)

# for idx_x, x in enumerate(np.arange(map.xlim[0], map.xlim[1] + map.dx, map.dx)):
#     for idx_y, y in enumerate(np.arange(map.ylim[0], map.xlim[1] + map.dy, map.dy)):
#         fx, fy, f = map.get_f([x,y])
#         f_all[idx_x, idx_y] = f
#         fx_all[idx_x, idx_y] = fx
#         fy_all[idx_x, idx_y] = fy

# map.plot(fx_all, 'fx')
# map.plot(fy_all, 'fy')
# map.plot(f_all, 'f_total')

# fig, ax = plt.subplots()
# q = ax.quiver(map.x, map.y, fx_all, fy_all)
# ax.quiverkey(q, X=0.3, Y=1.1, U=10,
#              label='Quiver key, length = 10', labelpos='E')
# plt.show()