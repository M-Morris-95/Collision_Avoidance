import numpy as np

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


        self.d_max = 2.5 # limit of influence from map
        self.d_min = min(self.dx, self.dy) # Stops F = inf

        # generate 2 2d grids for the x & y bounds
        self.x, self.y = np.meshgrid(np.arange(xlim[0], xlim[1]+self.dx, self.dx),
                                     np.arange(ylim[0], ylim[1]+self.dy, self.dy))

        self.z = np.zeros(map.shape)

        self.neg_fov = np.radians(-43)
        self.pos_fov = np.radians(43)

        self.r_min = 0.6
        self.r_max = 6.0

    def update(self, z, pos, dir):
        theta = np.arctan2(dir[1],dir[0])
        num_th = 25
        num_r = 41

        mul = np.asarray([1/self.dx, 1/self.dy])
        seen_arr = np.zeros((num_th)) + 1

        orig_angles = np.linspace(self.neg_fov, self.pos_fov, num_th)

        for r in np.linspace(self.r_min, self.r_max, num_r):
            angles = np.linspace(self.neg_fov, self.pos_fov, num_th)
            angles = angles * seen_arr
            angles = angles[angles != 0]

            n_seen = angles.shape[0]
            n_pos = np.tile(pos, [n_seen, 1]).T
            n_mul = np.tile(mul, [n_seen, 1]).T

            t = pol2cart(r, theta+angles) + n_pos
            t = (t * n_mul).round().astype(int).T
            t = t[t.min(axis=1)>=0, :]

            t=np.minimum(t, 74)

            for idx, xy in enumerate(t):
                if z[xy[1], xy[0]] == 1:
                    self.z[xy[1], xy[0]] = 1

                    seen_arr[np.argwhere(orig_angles == angles[idx])[0][0]] = 0

    def get_f(self, pos, type = 'square'):
        dx = pos[0] - self.x
        dy = pos[1] - self.y

        dxdy2 = np.square(dx) + np.square(dy)

        dxdy2[dxdy2 > self.d_max ** 2] = np.inf
        dxdy2[dxdy2 < self.d_min ** 2] = self.d_min ** 2

        if type == 'square':
            # self.f_map = np.divide(0.4*self.z, dxdy2)
            self.f_map = np.divide(1.0 * self.z, dxdy2)

        if type == 'gaussian':
            self.f_map = 5*np.exp(-np.sqrt(dxdy2)) * self.z


        thetas = np.arctan2(dx, dy)

        fx = np.multiply(np.sin(thetas), self.f_map).sum()
        fy = np.multiply(np.cos(thetas), self.f_map).sum()

        return fx, fy, self.f_map.sum()