import numpy as np
from . import config
from . import utils

class Wall:
  def __init__(self, inclination):
      self.inclination = inclination

  def point_on(self, distance):
      '''
      distance in meters returns [x,y] in meters along the wall direction from origin
      '''
      theta = np.radians(self.inclination)
      dx = distance * np.sin(theta)
      dy = distance * np.cos(theta)
      return dx, dy
  
  def direction(self):
      theta = np.radians(self.inclination)
      return np.array([np.sin(theta), np.cos(theta)])
  
  def normal_vector(self):
      theta = np.radians(self.inclination)
      return np.array([np.cos(theta), -np.sin(theta)])

  def distance(self, pos):
      return np.dot(np.array(pos), self.normal_vector())
  
  def draw(self, canvas):
      cw, ch, scale = config.cw, config.ch, config.scale
      d = np.sqrt((cw/scale)**2 + (ch/scale)**2)
      x1, y1 = utils.meter2pix(self.point_on(-d))
      x2, y2 = utils.meter2pix(self.point_on(d))
      normal = self.normal_vector()
      dx = -normal[0] * d * scale
      dy = normal[1] * d * scale
      x3, y3 = x2 + dx, y2 + dy
      x4, y4 = x1 + dx, y1 + dy
      canvas.create_polygon(x1, y1, x2, y2, x3, y3, x4, y4,
                            fill='#8B4513', stipple='gray25', outline='black')
      canvas.create_line(x1, y1, x2, y2, fill='black', width=2)