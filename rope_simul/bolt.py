import numpy as np
from . import config


class Bolt:
    def __init__(self, wall, distance):
        # distance along the wall from the origin (meters)
        self.wall = wall
        self.dist = distance

    def pos(self):
        # return (x,y) in meters
        return np.array(self.wall.point_on(self.dist))

    def draw(self, canvas, meter2pix, bolt_rad, color='black'):
        x, y = meter2pix(self.pos())
        canvas.create_oval(x-bolt_rad, y-bolt_rad, x+bolt_rad, y+bolt_rad, fill=color)

    @staticmethod
    def generate(wall, N=5, margin=3.0):
        """Generate N Bolt objects distributed along the visible portion of the wall.

        Parameters default to values from rope_simul.config when omitted.
        Returns list of Bolt instances.
        """

        cw, ch = config.cw, config.ch
        scale = config.scale
        Ox, Oy = config.Ox, config.Oy

        wall_dir = wall.direction()

        # compute canvas corners in meters relative to pivot (Ox,Oy)
        corners_px = [(0, 0), (cw, 0), (0, ch), (cw, ch)]
        corners_m = []
        for px, py in corners_px:
            x_m = (px - Ox) / scale
            y_m = (Oy - py) / scale
            corners_m.append(np.array([x_m, y_m]))

        projs = [np.dot(c, wall_dir) for c in corners_m]
        min_proj, max_proj = min(projs), max(projs)

        if max_proj - min_proj <= 2*margin or N <= 1:
            dists = [ (min_proj + max_proj) / 2.0 ]
        else:
            dists = list(np.linspace(min_proj + margin, max_proj - margin, N))

        bolts = [Bolt(wall, d) for d in dists]
        bolts.sort(key=lambda bolt: bolt.dist)
        return bolts
