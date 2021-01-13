import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator
import numpy as np

class map_2d():
    def __init__(self, size = [50,50], xlim = [0,10], ylim = [0,10]):
        self.size_x = size[0]
        self.size_y = size[0]


        self.range_x = xlim[1] - xlim[0]
        self.range_y = ylim[1] - ylim[0]

        self.dy = (ylim[1] - ylim[0]) / size[1]
        self.dx = (xlim[1] - xlim[0]) / size[0]

        # generate 2 2d grids for the x & y bounds
        self.y, self.x = np.mgrid[slice(ylim[0], ylim[1] + self.dy, self.dy),
                                  slice(xlim[0], xlim[1] + self.dx, self.dx)]



        # self.z = np.sin(self.x) ** 10 + np.cos(10 + self.y * self.x) * np.cos(self.x)
        self.z = np.zeros(self.x.shape)

        return None

    def fill_map(self):
        n_shapes = 25
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

        cf = ax0.contourf(self.x + self.dx / 2.,
                          self.y + self.dy / 2., to_plot)


        fig.colorbar(cf, ax=ax0)
        ax0.set_title(title)

        # adjust spacing between subplots so `ax1` title and `ax0` tick labels
        # don't overlap
        fig.tight_layout()

        plt.show()

    def get_f(self, pos):
        x = pos[0]
        y = pos[1]
        dx = []
        dy = []
        for i in range(self.size_x+1):
            dy.append(i - x)
            # dx.append(x - i)


        for j in range(self.size_y+1):
            dx.append(j - y)
            # dy.append(y - j)

        dx = np.asarray(dx)[np.newaxis, :]
        dy = np.asarray(dy)[:, np.newaxis]

        thetas = np.arctan2(dx, dy)
        self.dxdy = (np.tile(np.square(dx), (51, 1)) + np.tile(np.square(dy), (1, 51))) + 1e-5

        self.f_map = np.divide(self.z, self.dxdy)
        temp = np.minimum(self.f_map, np.zeros((51,51))+0.5) + 1e-5
        tot = temp.sum()


        # self.f_map = self.f_map / self.f_map.max()
        # temp = np.minimum(self.f_map, np.zeros((51, 51)) + 0.5)
        fx = np.multiply(np.sin(thetas), temp).sum()
        fy = np.multiply(np.cos(thetas), temp).sum()

        return fx, fy, tot

map = map_2d()
map.fill_map()
map.plot(map.z)


pos = np.multiply([9,3], 5)
pos_hist = []

f_all = np.zeros((51,51))
fx_all = np.zeros((51,51))
fy_all = np.zeros((51,51))
for i in range(51):
    for j in range(51):
        fx, fy, f = map.get_f([i,j])
        f_all[i,j] = f
        fx_all[i, j] = fx
        fy_all[i, j] = fy

map.plot(fx_all / fx_all.max(), 'fx')
map.plot(fy_all / fy_all.max(), 'fy')
map.plot(np.sqrt(f_all) / np.sqrt(f_all).max(), 'f_total')

# for i in range(25):
#
#     fx, fy = map.get_f(pos)
#     pos = (pos + np.asarray([fx, fy])).round().astype(int)
#     map.plot_dist(pos)
#     pos_hist.append(pos)





# # make these smaller to increase the resolution
# dx, dy = 0.05, 0.05
#
# # generate 2 2d grids for the x & y bounds
# y, x = np.mgrid[slice(1, 5 + dy, dy),
#                 slice(1, 5 + dx, dx)]
#
# z = np.sin(x)**10 + np.cos(10 + y*x) * np.cos(x)
#
# # x and y are bounds, so z should be the value *inside* those bounds.
# # Therefore, remove the last value from the z array.
# z = z[:-1, :-1]
# levels = MaxNLocator(nbins=15).tick_values(z.min(), z.max())
#
#
# # pick the desired colormap, sensible levels, and define a normalization
# # instance which takes data values and translates those into levels.
# cmap = plt.get_cmap('PiYG')
# norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)
#
# fig, ax0,  = plt.subplots(1,1)
#
#
#
#
# # contours are *point* based plots, so convert our bound into point
# # centers
# cf = ax0.contourf(x[:-1, :-1] + dx/2.,
#                   y[:-1, :-1] + dy/2., z, levels=levels,
#                   cmap=cmap)
# fig.colorbar(cf, ax=ax0)
# ax0.set_title('contourf with levels')
#
# # adjust spacing between subplots so `ax1` title and `ax0` tick labels
# # don't overlap
# fig.tight_layout()
#
# plt.show()