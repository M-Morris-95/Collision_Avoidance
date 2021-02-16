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

        self.d_max = 1.0 # limit of influence from map
        self.d_min = min(self.dx, self.dy) # Stops F = inf

        # generate 2 2d grids for the x & y bounds
        self.x, self.y = np.meshgrid(np.arange(xlim[0], xlim[1]+self.dx, self.dx),
                                     np.arange(ylim[0], ylim[1]+self.dy, self.dy))

        self.mxy = np.asarray([int(self.d_max / self.dx),int(self.d_max / self.dy)])


        self.z = np.zeros(map.shape)

        self.neg_fov = np.radians(-43)
        self.pos_fov = np.radians(43)

        self.r_min = 0.6
        self.r_max = 6.0

        self.dxm = np.tile(np.linspace(self.d_max, -self.d_max, int(self.d_max/self.dx*2+1)), (int(self.d_max/self.dx*2+1),1))
        self.dym = np.tile(np.linspace(self.d_max, - self.d_max, int(self.d_max/self.dx*2+1))[:, np.newaxis], (1,int(self.d_max/self.dx*2+1)))
        self.dxdy2 = np.square(self.dxm) + np.square(self.dym)
        self.dxdy2[self.dxdy2 < self.d_min ** 2] = self.d_min ** 2
        thetas = np.arctan2(self.dxm, self.dym)
        self.s_theta = np.sin(thetas)
        self.c_theta = np.cos(thetas)

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






        # theta = np.arctan2(dir[1],dir[0])
        # num_th = 25
        # num_r = 41
        #
        # mul = np.asarray([1/self.dx, 1/self.dy])
        # seen_arr = np.zeros((num_th)) + 1
        #
        # orig_angles = np.linspace(self.neg_fov, self.pos_fov, num_th)
        #
        # for r in np.linspace(self.r_min, self.r_max, num_r):
        #     angles = np.linspace(self.neg_fov, self.pos_fov, num_th)
        #     angles = angles * seen_arr
        #     angles = angles[angles != 0]
        #
        #     n_seen = angles.shape[0]
        #     n_pos = np.tile(pos, [n_seen, 1]).T
        #     n_mul = np.tile(mul, [n_seen, 1]).T
        #
        #     t = pol2cart(r, theta+angles) + n_pos
        #     t = (t * n_mul).round().astype(int).T
        #     t = t[t.min(axis=1)>=0, :]
        #
        #     t=np.minimum(t, self.size_x)
        #
        #     for idx, xy in enumerate(t):
        #         if z[xy[1], xy[0]] == 1:
        #             self.z[xy[1], xy[0]] = 1
        #
        #             seen_arr[np.argwhere(orig_angles == angles[idx])[0][0]] = 0

    def get_f(self, pos, type = 'square', return_sum = False):
        pos_b = (pos / np.asarray([self.dx, self.dy])).round(0).astype(int)
        min = np.max([pos_b - self.mxy, np.asarray([0,0])], 0)
        max = np.min([pos_b + self.mxy+1, np.asarray(self.z.shape)], 0)

        t_z = self.z[min[1]:max[1], min[0]:max[0]]

        if t_z.shape != self.dxdy2.shape:
            t = np.zeros(self.dxdy2.shape)
            t_max = np.max([self.mxy-pos_b, np.asarray([0, 0])], 0)
            t_min = np.min([np.asarray(self.z.shape) +(self.mxy)-pos_b, [11,11]],0)
            try:
                t[t_max[1]:t_min[1], t_max[0]:t_min[0]] = t_z
            except:
                pass
            t_z = t

        self.f_map = np.divide(t_z, self.dxdy2)

        fx = np.multiply(self.s_theta, self.f_map).sum()
        fy = np.multiply(self.c_theta, self.f_map).sum()

        if return_sum:
            return fx, fy, self.f_map.sum()

        return fx, fy

    def get_risk(self, pos_list, k=0, type = 'square', return_sum = False):
        tot = []
        for pos in pos_list[k:k+50]:
            pos_b = (pos / np.asarray([self.dx, self.dy])).round(0).astype(int)
            min = np.max([pos_b - self.mxy, np.asarray([0,0])], 0)
            max = np.min([pos_b + self.mxy+1, np.asarray(self.z.shape)], 0)

            t_z = self.z[min[1]:max[1], min[0]:max[0]]

            f_map = np.divide(t_z, self.dxdy2[0:t_z.shape[0], 0:t_z.shape[1]])

            tot.append(f_map.sum())

        return np.asarray(tot)