import numpy as np
import copy

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return np.asarray([x, y])

class realsense_map():
    def __init__(self, map, size = [75,75], xlim = [0,10], ylim = [0,10]):
        self.xlim = xlim
        self.ylim = ylim

        self.size_x = size[0]
        self.size_y = size[1]

        self.range_x = xlim[1] - xlim[0]
        self.range_y = ylim[1] - ylim[0]

        self.dx = (xlim[1] - xlim[0]) / size[0]
        self.dy = (ylim[1] - ylim[0]) / size[1]



        # generate 2 2d grids for the x & y bounds
        self.x, self.y = np.meshgrid(np.arange(xlim[0], xlim[1]+self.dx, self.dx),
                                     np.arange(ylim[0], ylim[1]+self.dy, self.dy))

        self.z = np.zeros(map.shape)

        self.neg_fov = np.radians(-43)
        self.pos_fov = np.radians(43)

        self.r_min = 0.6
        self.r_max = 6.0

    def update(self, z, pos, dir):
        r, thetas = cart2pol(self.x - pos[0], self.y - pos[1])
        thetas = thetas - np.arctan2(dir[1], dir[0])

        t_z = copy.copy(z)

        t_z[r > 6.0] = 0
        t_z[r < 0.6] = 0

        num = 25
        angles = np.linspace(-np.radians(43), np.radians(43), num)

        for t in range(num):
            try:
                a = np.logical_and.reduce((thetas != 0, thetas >= angles[t], thetas <= angles[t+1], t_z != 0))
                idx = np.argwhere(a)
                num = r[a].argmin()
                self.z[idx[num, 0], idx[num,1]] = 1
            except:
                pass

