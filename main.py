from draw import *
from realsense_map import *
from map import *
from ode_solver import *
import time
from path_finder import *
import numpy as np




do_plt = True


for _ in range(10):
    start = np.asarray([1.25,1.5])
    end = np.asarray([8.75,8.5])
    dt = 1/30

    xlim = [0,10]
    ylim = [0,10]
    size = [100,100]



    map = map_2d(size=size,
                 xlim=xlim,
                 ylim=ylim,
                 start=start + np.random.normal(0, 0.1, 2),
                 end=end + np.random.normal(0, 0.1, 2))

    map.fill_map()
    # map.custom_map()

    k = 1
    drone = realsense_map(size=size,
                          xlim=xlim,
                          ylim=ylim,
                          map=map.z)

    pos_ode = pos_solv(start=start,
                       end=end)

    path = path_finder(start=pos_ode.pos,
                        end=end,
                        sigma=0.25)

    route, options, goals = path.get_route(map = drone, start=pos_ode.pos, dt = dt, )
    drone.update(map.z, pos_ode.pos, pos_ode.vel)

    if do_plt:
        plot = anim(xlim = xlim, ylim = ylim, x = map.x, y = map.y)

    t=0
    start = time.time()
    while True:
        # used for plotting at a constant frame rate

        start_t = time.time()

        # update map
        drone.update(map.z, pos_ode.pos, pos_ode.vel)

        # decide whether make new route or keep old one
        risk, risk_eval = path.get_risk(route, drone, k=k, n=10, method = 2)

        if k>5 and (risk > 0 or k >= route.shape[0]-1):
            k = 1
            route, options, goals = path.get_route(map = drone, start=pos_ode.pos, dt = dt)

        else:
            k = k+1
        pos_ode.update(vel = (route[k] - pos_ode.pos) / dt, pos = route[k])

        t = t + dt

        # for i in range(28):
        #     plt.plot(options[i][:,0],options[i][:,1])
        #     plt.title(str(i))
        #     plt.xlim([0,10])
        #     plt.ylim([0,10])
        #     plt.show()

        if do_plt:
            plot.update(true_map = map.z,
                        drone_map = drone.z,
                        drone_pos = (pos_ode.pos[0], pos_ode.pos[1]),
                        drone_vel = (pos_ode.vel[0], pos_ode.vel[1]),
                        dt = dt,
                        route = route,
                        optional_routes = options,
                        path = pos_ode.pos_hist,
                        risk=risk,
                        risk_eval = risk_eval,
                        goals = goals)

            while time.time() - start_t < dt:
                time.sleep(0.00001)
            pygame.display.flip()

        if pos_ode.terminal() or t > 50:
            break

pygame.quit()
print(time.time()-start)





#
# cmap = plt.get_cmap('PiYG')



# fig, ax, = plt.subplots(1, 1, dpi = 400, figsize = [7,3.5])
# next = path_map + 2 * map.z
# next = (next - next.min())/(next.max()-next.min())
# levels = MaxNLocator(nbins=15).tick_values(next.min(), next.max())
# norm = BoundaryNorm(levels, ncolors=cmap.N, clip=True)
# cf = ax.contourf(drone.x, drone.y, next, antialiased = False)
# plt.tight_layout()
# plt.show()
#
# f_all = np.zeros(map.x.shape)
# fx_all = np.zeros(map.x.shape)
# fy_all = np.zeros(map.x.shape)
#
# drone.z = map.z
# for idx_x, x in enumerate(np.arange(drone.xlim[0], drone.xlim[1] + drone.dx, drone.dx)):
#     for idx_y, y in enumerate(np.arange(drone.ylim[0], drone.ylim[1] + drone.dy, drone.dy)):
#         fx, fy, f = drone.get_f([x,y], return_sum = True)
#
#         dxdy = pos_ode.end - np.asarray([x,y])
#
#         theta = np.arctan2(dxdy[1], dxdy[0])
#
#         fx_goal = pos_ode.f_goal * np.cos(theta)
#         fy_goal = pos_ode.f_goal * np.sin(theta)
#
#         f_all[idx_y, idx_x] = f
#         fx_all[idx_y, idx_x] = fx + fx_goal
#         fy_all[idx_y, idx_x] = fy + fy_goal
#
#
# fig, ax = plt.subplots()
# q = ax.quiver(drone.y, drone.x, fx_all, fy_all)
# ax.quiverkey(q, X=0.3, Y=1.1, U=10,
#              label='Quiver key, length = 10', labelpos='E')
# plt.show()
# for x in range(10):
#
#     plot = anim(xlim = xlim, ylim = ylim)
#
#     pos = np.asarray(pos_ode.pos_hist)
#     vel = np.asarray(pos_ode.vel_hist)
#     for i in range(pos.shape[0]):
#         plot.screen.fill(pygame.Color("white"))
#         plot.apply_map(map.x, map.y, map.z)
#         plot.drone_pos(pos[i,0],pos[i,1], vel[i,0], vel[i,1], dt = dt)
#         time.sleep(dt)
#         pygame.display.flip()
#     time.sleep(1)
#
