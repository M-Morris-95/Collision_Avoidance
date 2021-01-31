import copy
from draw import *
from realsense_map import *
from map import *
from ode_solver import *
from scipy import signal

start = np.asarray([1.5,1.5])
end = np.asarray([8.5,1.5])

xlim = [0,10]
ylim = [0,10]

map = map_2d(xlim = xlim,
             ylim = ylim,
             start=start,
             end=end)

drone = realsense_map(xlim = xlim,
                  ylim = ylim,
                  map = map.z)

pos_ode = pos_solv(start=start,
                   end=end)

# map.fill_map()
map.custom_map()
map.plot(map.z)

for run in range(5):
    pos_ode = pos_solv(start=start, end=end)

    pos_ode.noise_param['sigma'] = run * 0.1
    drone.update(map.z, pos_ode.pos, pos_ode.vel)

    plot = anim(xlim = xlim, ylim = ylim)
    plot.apply_map(map.x, map.y, map.z)




    for i in range(200):
        pos_ode.next_pos(drone)
        drone.update(map.z, pos_ode.pos, pos_ode.vel)

        plot.drone_pos(pos_ode.pos[0], pos_ode.pos[1],
                       pos_ode.vel[0], pos_ode.vel[1], dt=0.05)
        time.sleep(0.05)
        if pos_ode.terminal():
            break

    pos = (np.asarray(pos_ode.pos_hist) / np.asarray([map.dx, map.dy])).astype(int)
    pos = np.minimum(pos, 75)

    path_map = copy.copy(drone.z)
    for i in range(pos.shape[0]):
        path_map[pos[i][1], pos[i][0]] = -1

    if pos_ode.terminal():
        break

map.z[0,0] = -1

cmap = plt.get_cmap('PiYG')



fig, ax, = plt.subplots(1, 1, dpi = 400, figsize = [7,3.5])
next = path_map + 2 * map.z
next = (next - next.min())/(next.max()-next.min())
levels = MaxNLocator(nbins=15).tick_values(next.min(), next.max())
norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)
cf = ax.contourf(drone.x, drone.y, next, antialiased = False)
plt.tight_layout()
plt.show()

f_all = np.zeros(map.x.shape)
fx_all = np.zeros(map.x.shape)
fy_all = np.zeros(map.x.shape)

for idx_x, x in enumerate(np.arange(drone.xlim[0], drone.xlim[1] + drone.dx, drone.dx)):
    for idx_y, y in enumerate(np.arange(drone.ylim[0], drone.ylim[1] + drone.dy, drone.dy)):
        fx, fy, f = drone.get_f([x,y])

        dxdy = pos_ode.end - np.asarray([x,y])

        theta = np.arctan2(dxdy[1], dxdy[0])

        fx_goal = pos_ode.f_goal * np.cos(theta)
        fy_goal = pos_ode.f_goal * np.sin(theta)

        f_all[idx_y, idx_x] = f
        fx_all[idx_y, idx_x] = fx + fx_goal
        fy_all[idx_y, idx_x] = fy + fy_goal


fig, ax = plt.subplots()
q = ax.quiver(drone.y, drone.x, fx_all, fy_all)
ax.quiverkey(q, X=0.3, Y=1.1, U=10,
             label='Quiver key, length = 10', labelpos='E')
plt.show()
for x in range(10):
    plot = anim(xlim = xlim, ylim = ylim)
    plot.apply_map(map.x, map.y, map.z)
    pos = np.asarray(pos_ode.pos_hist)
    vel = np.asarray(pos_ode.vel_hist)
    for i in range(pos.shape[0]):
        plot.drone_pos(pos[i,0],pos[i,1], vel[i,0], vel[i,1], dt = 0.05)
        time.sleep(0.05)

    time.sleep(1)
pygame.quit()
