import numpy as np
from numpy.linalg import norm

from . import config

class Person:
    def __init__(self, state=None, rad=0.0, mass=0.0, bolt=None, grabbed=False):
        self.rad = rad
        self.mass = mass
        self.bolt = bolt
        self.grabbed = grabbed
        if state is None:
            self.state = [0.0, 0.0, 0.0, 0.0]
        else:
            self.state = state
        if self.bolt is None:
            raise ValueError('Person has no bolt attached')
        self.bolt = bolt

    def initialize_on_wall(self, wall, dist):
        x = self.bolt.pos()[0] + wall.point_on(dist)[0] + wall.normal_vector()[0] * (1.01 * self.rad / config.scale)
        y = self.bolt.pos()[1] + wall.point_on(dist)[1] + wall.normal_vector()[1] * (1.01 * self.rad / config.scale)
        self.state = [x, y, 0.0, 0.0]

    def initialize_on_floor(self): #Still missing bottom of the wall functionality
        floor_y = (config.Oy - config.ch + self.rad) / config.scale
        x = self.bolt.pos()[0]
        self.state = [x, floor_y, 0.0, 0.0]

    def collision(self, wall):
        pos = np.array(self.state[:2])
        vel = np.array(self.state[2:])
        d_signed = wall.distance(pos)
        radius_m = self.rad / config.scale
        penetration = radius_m - d_signed
        if penetration > 0.0:
            normal = wall.normal_vector()
            vel_n = np.dot(vel, normal)
            e = 0.25
            vel_t = vel - vel_n * normal
            vel_post = vel_t - e * vel_n * normal
            self.state[2] = vel_post[0]
            self.state[3] = vel_post[1]
            self.state[0] += normal[0] * (penetration + 1e-4)
            self.state[1] += normal[1] * (penetration + 1e-4)

        # Floor collision (horizontal floor at the bottom of canvas)
        # floor_y matches initialize_on_floor calculation
        floor_y = (config.Oy - config.ch + self.rad) / config.scale
        # recompute pos/vel because wall collision may have modified state
        pos = np.array(self.state[:2])
        vel = np.array(self.state[2:])
        d_floor = pos[1] - floor_y
        penetration_floor = radius_m - d_floor
        if penetration_floor > 0.0:
            normal = np.array([0.0, 1.0])
            vel_n = np.dot(vel, normal)
            e_floor = 0.05
            vel_t = vel - vel_n * normal
            vel_post = vel_t - e_floor * vel_n * normal
            self.state[2] = vel_post[0]
            self.state[3] = vel_post[1]
            self.state[0] += normal[0] * (penetration_floor + 1e-4)
            self.state[1] += normal[1] * (penetration_floor + 1e-4)
