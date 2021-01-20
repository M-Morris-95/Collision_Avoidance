import pygame
import numpy as np
from map import *
from ode_solver import *

class anim():
    def __init__(self, xlim = [0,10], ylim = [0,10]):
        pygame.init()

        # Set up the drawing window
        self.xlim = xlim
        self.ylim = ylim

        self.n_pix = np.asarray([500, 500])
        self.padding = np.asarray([50, 50])
        self.screen = pygame.display.set_mode(self.n_pix+self.padding*2)

        dx = self.n_pix[0] / (xlim[1] - xlim[0])
        dy = self.n_pix[1] / (ylim[1] - ylim[0])

        self.dxdy = np.asarray([dx, dy])

    def plot_sq(self, pos, sq_size, color = (0,0,255)):

        sq_size = sq_size * self.dxdy
        pos = pos * self.dxdy
        w = int(sq_size[0])
        h = int(sq_size[1])

        l = int(pos[0] - w / 2)
        t = self.n_pix[1] - int(pos[1] + w / 2)

        pygame.draw.rect(self.screen, color, pygame.Rect(l, t, w, h))

    def apply_map(self, x, y, z):
        tx, ty = np.indices(z.shape)
        tx = tx.reshape(-1)
        ty = ty.reshape(-1)
        color = []
        for pos in np.asarray([tx,ty]).T:
            color.append(255 * z[pos[0], pos[1]])

        color = np.asarray(color).astype(int)
        color = np.maximum(color, 0)

        px = (x[tx, ty] * self.dxdy[0]).astype(int)
        py = (self.n_pix[1] - (y[tx, ty] * self.dxdy[1])).astype(int)

        pxpy = np.asarray([px,py]).T
        for idx,pos in enumerate(pxpy):
            pos = pos+self.padding
            pygame.draw.rect(self.screen, (color[idx],0,0), pygame.Rect(pos[0], pos[1], 7, 7))


        return True

    def run(self, map):

        # Run until the user asks to quit
        running = True
        while running:

            # Did the user click the window close button?
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Fill the background with white
            self.screen.fill((255, 255, 255))

            # Draw a solid blue circle in the center
            self.apply_map(map.x, map.y, map.z)

            # pygame.draw.circle(screen, (0, 0, 255), (250, 250), 75)

            # Flip the display
            pygame.display.flip()

        # Done! Time to quit.
        pygame.quit()

start = np.asarray([1,1])
end = np.asarray([7,5])
xlim = [0,10]
ylim = [0,10]

map = map_2d(xlim = xlim,
             ylim = ylim,
             start=start,
             end=end)

map.fill_map()
map.plot(map.z)


plot = anim(xlim = xlim, ylim = ylim)

plot.run(map)