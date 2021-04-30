import numpy as np
import copy
from scipy.interpolate import splev, splrep
import matplotlib.pyplot as plt
from matplotlib import ticker, cm

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return np.asarray([x, y])

def smooth(route, tol=0.05):
    dist_mat = np.zeros((route.shape[0], route.shape[0])) + 10
    for i in range(route.shape[0]):
        err = np.tile(route[i], (route.shape[0] - i, 1)) - route[i:]
        dist_mat[i, i:] = np.sqrt(np.sum(np.square(err),1))

    dist_mat = dist_mat < tol

    new_route = [0]
    while new_route[-1] < route.shape[0]-1:
        new_route.append(np.argwhere(dist_mat[new_route[-1]]).max())

    return route[new_route]

def route_length(route, slow=False):
    if slow:
        return np.sum(np.sqrt(np.sum(np.square(np.diff(route, 1, 0)),1)))
    else:
        return np.sqrt(np.sum(np.square(np.diff(route[:2], 1, 0)),1)) * len(route)


class path_finder():
    def __init__(self, start = np.array([1,5]), end = np.array([9,5]), sigma = 0.5, speed=2):
        self.end = end

        self.tol = 0.5
        self.mass = 1.0
        self.f_goal = 15
        self.max_speed = 1

        self.dxdy2 = None
        self.speed = speed

    def euler_step(self, position, velocity, force, dt):
        # This method takes a position, velocity, force, and timestep and returns the next position, velocity and
        # distance travelled. This uses euler integration although RK4 or other may be better.

        velocity = velocity + dt*force/self.mass
        speed = np.sqrt(np.square(velocity).sum())

        velocity = velocity * self.max_speed/speed
        position = position + velocity * dt
        return position, velocity

    def route_to_goal(self, start, finish, velocity, map, dt=0.05, max_time=30):
        # Given a start and a method this route will return a low risk route between them. If a maximum length or time
        # are specified then the route will not exceed these, this may mean that the route does not reach the goal. The
        # method returns an array of positions, velocities, the length of the route, and whether the route reaches the
        # goal.

        position = [start]
        velocity = [velocity]

        for _ in range(int(max_time / dt)):
            terminal = False
            force = self.get_f(map, position[-1], f_goal=True, goal=finish)
            step_position, step_velocity = self.euler_step(position[-1], velocity[-1], force, dt)
            position.append(step_position)
            velocity.append(step_velocity)

            if np.sqrt(np.sum(np.square(position[-1] - finish))) <= self.tol:
                terminal = True
                break

        return np.asarray(position), np.asarray(velocity), terminal

    def get_route(self, map, start, velocity=np.asarray([0,0]), dt=0.05, num_points = [16, 8], dist = [2.25, 1]):
        # generate routes to points in a circle around the goal and then from these points to the goal. Find the
        # shortest of these routes and return it

        # generate circle of goals around target position
        goals = self.end[np.newaxis,:]
        for i in range(len(num_points)):
            more_goals = np.tile(self.end.T, [num_points[i], 1]) + pol2cart(rho=dist[i], phi=np.linspace(0, np.pi * 2, num_points[i])).T
            goals = np.concatenate([goals, more_goals])

        # remove goals that are not inside the map
        goals = goals[(goals>9.5).any(1)==False]
        goals = goals[(goals<0.5).any(1)==False]

        # Generate routes to each goal position, and from those to the target
        options = []
        l = []
        r = []
        t = []
        for goal in goals:
            position_l1, velocity_l1, _ = self.route_to_goal(start, goal, velocity, map, dt=dt)
            position_l2, velocity_l2, terminal = self.route_to_goal(position_l1[-1], self.end, velocity_l1[-1], map, dt=dt)
            smoothed = smooth(np.concatenate([position_l1, position_l2]), tol=0.1)
            length = route_length(smoothed)
            risk = self.get_risk(smoothed, map)

            l.append(length[0])
            r.append(risk[0])
            t.append(terminal)

            options.append(smoothed)

        # decide which route is best
        l = np.round(l, 2)
        l = np.asarray(l) #length
        r = np.asarray(r) #risk
        t = np.asarray(t) #does it get to the goal
        ratings = np.logical_and(t == 1, r == 0) * 1/l # hacky formula
        best_route = np.argmax(ratings)

        return options[best_route], options, goals

    def gen_dxdy2(self, map, d_max=1.0):
        # Generate matrix of distances, angles, and mxy which i don't remember whats its for. There are all used in the
        # get F method, calling this once instead of at every step speeds up calculation

        d_min = np.min([map.dx, map.dy])
        dxm = np.tile(np.linspace(d_max, - d_max, int(d_max/map.dx*2+1)), (int(d_max/map.dx*2+1),1))
        dym = np.tile(np.linspace(d_max, - d_max, int(d_max/map.dx*2+1))[:, np.newaxis], (1,int(d_max/map.dx*2+1)))
        self.dxdy2 = np.square(dxm) + np.square(dym)
        self.dxdy2[self.dxdy2 < d_min ** 2] = d_min ** 2

        thetas = np.arctan2(dxm, dym)
        self.s_theta = np.sin(thetas)
        self.c_theta = np.cos(thetas)

        self.mxy = np.asarray([int(d_max / map.dx),int(d_max / map.dy)])

    def get_f(self, map, pos, d_max=1.0, return_sum=False, f_goal=False, goal=None):
        # this method calculates the force on the drone at a point in the map. It uses preset matrix of distances to
        # speed up calculations

        # generate distances matrix
        if not isinstance(self.dxdy2, np.ndarray):
            self.gen_dxdy2(map, d_max=d_max)

        # find index of position
        pos_b = (pos / np.asarray([map.dx, map.dy])).round(0).astype(int)


        min = np.max([pos_b - self.mxy, np.asarray([0,0])], 0)
        max = np.min([pos_b + self.mxy+1, np.asarray(map.z.shape)], 0)

        t_z = map.z[min[1]:max[1], min[0]:max[0]]

        if t_z.shape != self.dxdy2.shape:
            t = np.zeros(self.dxdy2.shape)
            t_max = np.max([self.mxy-pos_b, np.asarray([0, 0])], 0)
            t_min = np.min([np.asarray(map.z.shape) +self.mxy-pos_b, [11,11]],0)
            try:
                t[t_max[1]:t_min[1], t_max[0]:t_min[0]] = t_z
            except:
                pass
            t_z = t

        self.f_map = np.divide(t_z, self.dxdy2)

        fx = np.multiply(self.s_theta, self.f_map).sum()
        fy = np.multiply(self.c_theta, self.f_map).sum()

        if f_goal:
            dxdy = goal - pos
            theta = np.arctan2(dxdy[1], dxdy[0])

            fx_goal = self.f_goal * np.cos(theta)
            fy_goal = self.f_goal * np.sin(theta)

            fx = fx + fx_goal
            fy = fy + fy_goal

        return np.asarray([fx, fy])

    def get_risk(self, pos_list, map, k=0, n = 100, method=2, plot = False):
        # returns the risk of a route at n points along it.
        idxs = np.linspace(k, pos_list.shape[0]-1, np.min(np.asarray([pos_list.shape[0]-k, n]))).astype(int)

        if method == 1:
            # method based on forces
            tot = []
            for idx in idxs:
                F = self.get_f(map, pos_list[idx])
                F = np.sqrt(np.sum(np.square(F)))
                tot.append(F)

            risk = np.asarray(tot).sum()
            z = None
        if method == 2:
            # method based on checking if anything a fixed distance from drone is within range
            risk_dim = 0.25

            mins = pos_list[idxs] - risk_dim
            mins = np.round(mins * 10).astype(int)
            maxs = pos_list[idxs] + risk_dim
            maxs = np.round(maxs * 10).astype(int)

            x = copy.copy((map.x*10).astype(int))
            y = copy.copy((map.y*10).astype(int))
            z = np.zeros(map.z.shape)

            for i in range(mins.shape[0]):
                try:
                    x_y_min = np.where(np.logical_and(mins[i,0] == x, mins[i,1] == y))
                    x_y_max = np.where(np.logical_and(maxs[i,0] == x, maxs[i,1] == y))
                    z[x_y_min[0][0]:x_y_max[0][0],x_y_min[1][0]:x_y_max[1][0]] = 1
                except:
                    pass

            risk = (z*map.z).sum()

        # plot risk map
        if plot:
            fig, ax, = plt.subplots(1, 1, dpi = 400, figsize = [7,3.5])
            cs = ax.contourf(x, y, z + 2*map.z + 2*map.z*z)
            plt.title(str(risk))
            plt.show()

        return risk, z

    def resample(self, route, dt=0.005):
        # resample the route to simplify it, currently untested
        try:
            route_len = np.sum(np.sqrt(np.sum((np.square(np.diff(route, axis=0))), 1)))

            num_time_steps = int(route_len / self.speed / dt)
            end_time = (route.shape[0] - 1) * dt,

            crude_t = np.linspace(0, end_time, 50)
            t = np.linspace(0, (route.shape[0]-1) * dt, route.shape[0])

            splx = splrep(t, route[:, 0])
            sply = splrep(t, route[:, 1])

            x = splev(crude_t, splx)
            y = splev(crude_t, sply)

            fine_t = np.linspace(0, end_time, num_time_steps)
            splx = splrep(crude_t, x)
            sply = splrep(crude_t, y)

            x = splev(fine_t, splx)
            y = splev(fine_t, sply)

            route = np.asarray([x, y]).T.squeeze()
        except:
            pass
        return route
