import pygame
import pygame.gfxdraw
import numpy as np
from map import *
from ode_solver import *
import time

class anim():
    def __init__(self, xlim = [0,10], ylim = [0,10], x = 0, y = 0):
        pygame.init()

        # Set up the drawing window
        self.xlim = xlim
        self.ylim = ylim

        self.n_pix = np.asarray([500, 500])
        self.padding = np.asarray([50, 50])
        self.screen = pygame.display.set_mode(size=(self.n_pix+self.padding*2))

        dx = self.n_pix[0] / (xlim[1] - xlim[0])
        dy = self.n_pix[1] / (ylim[1] - ylim[0])

        self.dxdy = np.asarray([dx, dy])
        self.screen.fill((255, 255, 255))

        self.x = x
        self.y = y


    def plot_sq(self, pos, sq_size, color = (0,0,255)):

        sq_size = sq_size * self.dxdy
        pos = pos * self.dxdy
        w = int(sq_size[0])
        h = int(sq_size[1])

        l = int(pos[0] - w / 2)
        t = self.n_pix[1] - int(pos[1] + w / 2)

        pygame.draw.rect(self.screen, color, pygame.Rect(l, t, w, h))

    def apply_map(self, x, y, z, color = (255,0,0)):
        tx, ty = np.indices(z.shape)

        tx = tx[z==1].reshape(-1)
        ty = ty[z==1].reshape(-1)

        px = (x[tx, ty] * self.dxdy[0]).astype(int)
        py = (self.n_pix[1] - (y[tx, ty] * self.dxdy[1])).astype(int)

        pxpy = np.asarray([px,py]).T
        for idx,pos in enumerate(pxpy):
            pos = pos+self.padding
            pygame.draw.rect(surface = self.screen,
                             color = color,
                             rect = pygame.Rect(pos[0], pos[1], 7, 7),
                             width = 0)


        return True

    def update(self, true_map, drone_map, drone_pos, drone_vel, dt, route, path):
        self.screen.fill(pygame.Color("white"))
        self.apply_map(self.x, self.y, true_map, (255, 200, 200))
        self.apply_map(self.x, self.y, drone_map, (255, 0, 0))
        self.drone_pos(drone_pos[0], drone_pos[1], drone_vel[0], drone_vel[1], dt = dt)

        x, y = self.xy2pix(route[:, 0], route[:, 1])
        for px, py in zip(x, y):
            pygame.draw.circle(surface=self.screen,
                               color=(0, 0, 255),
                               center=(px, py),
                               radius=2)
        path = np.asarray(path)
        x, y = self.xy2pix(path[:, 0], path[:, 1])
        for px, py in zip(x, y):
            pygame.draw.circle(surface=self.screen,
                               color=(0, 255, 255),
                               center=(px, py),
                               radius=2)

    def xy2pix(self, x, y=None):
        x = (x - self.xlim[0]) / (self.xlim[1] - self.xlim[0])
        y = (y - self.ylim[0]) / (self.ylim[1] - self.ylim[0])

        x = self.n_pix[0] * x + self.padding[0]
        y = self.n_pix[1] - self.n_pix[1]*y + self.padding[1]



        return x.astype(int), y.astype(int)

    def drone_pos(self, px, py, vx, vy, dt = 0.05):
        px_1, py_1 = self.xy2pix(px, py)

        pygame.draw.circle(surface=self.screen,
                         color=(0, 255, 0),
                         center = (px_1, py_1),
                         radius = 2)

        angle = np.arctan2(vx, vy)

        if angle < 0.5:
            pass


        pygame.gfxdraw.arc(self.screen,
                        px_1,
                        py_1,
                        int(0.6*self.dxdy[0]),
                        int(np.degrees(angle)-43-90),
                        int(np.degrees(angle)+43-90),
                        (255, 0, 255),
                        )

        pygame.gfxdraw.arc(self.screen,
                        px_1,
                        py_1,
                        int(6*self.dxdy[0]),
                        int(np.degrees(angle)-43-90),
                        int(np.degrees(angle)+43-90),
                        (255, 0, 255),
                        )



        lx1, ly1, = self.xy2pix(px + 0.6*np.sin(angle + np.radians(43)), py + 0.6*np.cos(angle + np.radians(43)))
        lx2, ly2, = self.xy2pix(px + 6 * np.sin(angle + np.radians(43)), py + 6 * np.cos(angle + np.radians(43)))
        # pygame.draw.line(surface=self.screen,
        #                  color = (255,0,255),
        #                  start_pos=(lx1, ly1),
        #                  end_pos=(lx2, ly2),
        #                 width = 2
        #                  )

        pygame.gfxdraw.line(self.screen,
                            lx1,
                            ly1,
                            lx2,
                            ly2,
                            (255,0,255),
                            )

        lx1, ly1, = self.xy2pix(px + 0.6*np.sin(angle + np.radians(-43)), py + 0.6*np.cos(angle + np.radians(-43)))
        lx2, ly2, = self.xy2pix(px + 6 * np.sin(angle + np.radians(-43)), py + 6 * np.cos(angle + np.radians(-43)))
        pygame.gfxdraw.line(self.screen,
                            lx1,
                            ly1,
                            lx2,
                            ly2,
                            (255,0,255),
                            )



        return True


# start = np.asarray([1,1])
# end = np.asarray([7,5])
# xlim = [0,10]
# ylim = [0,10]
#
# map = map_2d(xlim = xlim,
#              ylim = ylim,
#              start=start,
#              end=end)
#
# map.fill_map()
# map.plot(map.z)
#
#
# plot = anim(xlim = xlim, ylim = ylim)
# plot.apply_map(map.x, map.y, map.z)
#
# plot.drone_pos(2, 2, 1, 1)
# time.sleep(3)
# pygame.quit()