import numpy as np
from . import config


class Bolt:
    def __init__(self, wall, distance):
        # distance along the wall from the pivot (meters)
        self.wall = wall
        self.dist = distance

    def pos(self):
        # return (x,y) in meters
        return np.array(self.wall.point_on(self.dist))

    def draw(self, canvas, meter2pix, bolt_rad, color='black'):
        x, y = meter2pix(self.pos())
        canvas.create_oval(x-bolt_rad, y-bolt_rad, x+bolt_rad, y+bolt_rad, fill=color)

    @staticmethod
    def generate(wall, N=5, cw=None, ch=None, scale=None, Ox=None, Oy=None, margin=2.0):
        """Generate N Bolt objects distributed along the visible portion of the wall.

        Parameters default to values from rope_simul.config when omitted.
        Returns list of Bolt instances.
        """
        from . import config

        if cw is None: cw = config.cw
        if ch is None: ch = config.ch
        if scale is None: scale = config.scale
        if Ox is None: Ox = config.Ox
        if Oy is None: Oy = config.Oy

        theta = np.radians(wall.inclination)
        wall_dir = np.array([np.sin(theta), np.cos(theta)])

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


    @staticmethod
    def create_scene(wall, N=5, slack=None, mass=None, rad=None, rope_specs=None):
        """Create bolts, belayer, climber and Rope for a default scene.

        Returns (belayer, climber, rope, bolts).
        """
        from .person import Person
        from .rope import Rope
        from . import config

        if slack is None:
            slack = config.L
        if mass is None:
            mass = config.m
        if rad is None:
            rad = config.rad
        if rope_specs is None:
            rope_specs = (config.k1, config.k3)

        bolts = Bolt.generate(wall, N=N)
        belayer = Person(rad=rad, mass=mass)
        climber = Person(rad=rad, mass=mass)

        # position belayer just off the wall at the bottommost bolt and climber at the topmost bolt
        if bolts:
            bottom_bolt = bolts[0]
            top_bolt = bolts[-1]
            belayer.attach_bolt(bottom_bolt)
            climber.attach_bolt(top_bolt)
            belayer.initialize_on_wall(wall, dist=bottom_bolt.dist)
            climber.initialize_on_wall(wall, dist=top_bolt.dist)
        else:
            # fallback to reasonable positions
            belayer.initialize_on_floor(wall)
            climber.initialize_on_wall(wall, dist=0.0)

        rope = Rope(rope_specs, belayer, climber, bolts=bolts, slack=slack)
        return belayer, climber, rope, bolts

    @classmethod
    def generate_along_wall(cls, wall, N=5, margin=2.0):
        """Generate N bolts along the visible portion of the wall within the canvas.
        Bolts are placed by projecting the canvas corners onto the wall direction
        and spacing N distances evenly between the visible min and max, leaving a margin.
        Returns a list of Bolt instances.
        """
        cw, ch, scale = config.cw, config.ch, config.scale
        theta = np.radians(config.inclination)
        wall_dir = np.array([np.sin(theta), np.cos(theta)])
        Ox, Oy = config.Ox, config.Oy
        corners_px = [(0, 0), (cw, 0), (0, ch), (cw, ch)]
        corners_m = []
        for px, py in corners_px:
            x_m = (px - Ox) / scale
            y_m = (Oy - py) / scale
            corners_m.append(np.array([x_m, y_m]))
        projs = [np.dot(c, wall_dir) for c in corners_m]
        min_proj, max_proj = min(projs), max(projs)
        if max_proj - min_proj <= 2 * margin or N <= 1:
            dists = [ (min_proj + max_proj) / 2.0 ]
        else:
            dists = list(np.linspace(min_proj + margin, max_proj - margin, N))
        return [cls(wall, d) for d in dists]
