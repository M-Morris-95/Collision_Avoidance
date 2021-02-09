import numpy as np
import matplotlib.pyplot as plt

class drone():
    def __init__(self, mass = 1.0, start = np.asarray([1,1]), goal = np.asarray([9,9]), dt = 0.05):
        self.mass = mass
        self.pos = [start]
        self.goal = goal
        self.vel = [np.asarray([0,0])]
        self.dt = dt

        self.noise = np.random.normal([0,0],[3,3], 2)
        self.noise_param = {'mu': 0, 'sigma': 0.25, 'decay_period': 25}

    def ornstein_uhlenbeck_noise(self):
        self.noise = self.noise * (1 - 1 / self.noise_param['decay_period']) + np.random.normal(self.noise_param['mu'],
                                                                                                self.noise_param[
                                                                                                    'sigma'], 2)

    def update_pos(self):
        vel_n = self.vel[-1]+self.dt*self.get_f()/self.mass

        if np.sqrt(np.sum(np.square(vel_n))) > 5:
            vel_n = vel_n / (np.sqrt(np.sum(np.square(vel_n))) / 5)

        pos_n = self.pos[-1]+self.dt*vel_n

        self.pos.append(pos_n)
        self.vel.append(vel_n)

        return True

    def get_f(self, f_goal = 10):
        dxdy = self.goal - self.pos[-1] + self.noise

        theta = np.arctan2(dxdy[1], dxdy[0])

        fx_goal = f_goal * np.cos(theta)
        fy_goal = f_goal * np.sin(theta)

        self.ornstein_uhlenbeck_noise()
        return np.asarray([fx_goal, fy_goal])

    def terminal(self, tol = 0.5):
        if np.sqrt(np.sum(np.square(self.goal - self.pos[-1]))) < tol:
            return True
        else:
            return False

start = np.asarray([1,1])
end = np.asarray([9,9])

model = drone()
for k in range(1000):
    model.update_pos()
    if model.terminal():
        break

plt.plot(np.asarray(model.pos)[:,0], np.asarray(model.pos)[:,1])
plt.show()

# noise = []
# for i in range(100):
#     noise.append(model.noise)
#     model.ornstein_uhlenbeck_noise()
#
# plt.plot(np.asarray(noise)[:,0], np.asarray(noise)[:,1])
# plt.show()