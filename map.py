import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator
import numpy as np

class map_2d():
    def __init__(self, size = [75,75], xlim = [0,10], ylim = [0,10], start = np.asarray([1,5]), end = np.asarray([9,5])):
        self.xlim = xlim
        self.ylim = ylim

        self.start = start
        self.end = end

        self.size_x = size[0]
        self.size_y = size[1]

        self.range_x = xlim[1] - xlim[0]
        self.range_y = ylim[1] - ylim[0]

        self.dx = (xlim[1] - xlim[0]) / size[0]
        self.dy = (ylim[1] - ylim[0]) / size[1]


        self.d_max = 2.5 # limit of influence from map
        self.d_min = min(self.dx, self.dy) # Stops F = inf

        # generate 2 2d grids for the x & y bounds
        self.x, self.y = np.meshgrid(np.arange(xlim[0], xlim[1]+self.dx, self.dx),
                                     np.arange(ylim[0], ylim[1]+self.dy, self.dy))

        self.z = np.zeros(self.x.shape) + np.random.normal(0, 0.000, self.x.shape)

        self.z[:2, :] = 1
        self.z[-2:, :] = 1
        self.z[:, :2] = 1
        self.z[:, -2:] = 1

        return None

    def fill_map(self):
        n_shapes = 20
        size_of_shapes = 10

        for _ in range(n_shapes):
            pos = [(self.size_x*np.random.random(1)).astype(int),
                   (self.size_y*np.random.random(1)).astype(int)]

            size = (size_of_shapes*np.random.random(1)).astype(int)[0]

            self.z[int(pos[0]-size/2) : int(pos[0]+size/2), int(pos[1]-size/2) : int(pos[1]+size/2)] = 1

        self.add_shape(self.start, 1)
        self.add_shape(self.end, 1)

    def add_shape(self, pos, size, val=0):
        pos = (pos / np.asarray([self.dx,self.dy])).round().astype(int)
        num_vox = np.asarray([-0.5*size/self.dx, 0.5*size/self.dy]).round().astype(int)
        self.z[pos[1] + num_vox[0]:pos[1] + num_vox[1], pos[0] + num_vox[0]:pos[0] + num_vox[1]] = val

    def custom_map(self):
        x = np.asarray([[1.8, 2.2],
             [1.8, 2.2],
             [4.8, 5.2],
             [6.8, 7.2],
             [6.8, 7.2]]) / self.dx
        y = np.asarray([[0,4],
             [6,10],
             [4,6],
             [0,4],
             [6,10]]) / self.dy
        x = x.astype(int)
        y = y.astype(int)

        for i in range(x.shape[0]):
            x0 = x[i, 0]
            x1 = x[i, 1]
            y0 = y[i, 0]
            y1 = y[i, 1]
            self.z[y0:y1, x0:x1]=1









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

        ax0.set_xlabel('x')
        ax0.set_ylabel('y')

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