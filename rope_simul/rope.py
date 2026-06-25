# from .utils import SOMETHING
# from .conig import SOMETHING ELSE
import numpy as np
from numpy.linalg import norm


class Rope:
  def __init__(self, specifics, belayer, climber, bolts=None, slack=0.0):
    self.k1, self.k3 = specifics
    self.belayer = belayer
    self.climber = climber
    # list of Bolt objects (can be empty)
    self.bolts = bolts if bolts is not None else []
    self.slack = slack
    # l0 is how much total rope is in the system
    self.l0 = self.initial_length()

  def initial_length(self):
    if self.bolts:
      bottom = self.bolts[0].pos()
      top = self.bolts[-1].pos()
      return (norm(np.array(self.belayer.state[:2]) - bottom)
              + norm(top - bottom)
              + norm(np.array(self.climber.state[:2]) - top)
              + self.slack)
    points = [np.array(self.belayer.state[:2])]
    for bolt in self.bolts:
      points.append(np.array(bolt.pos()))
    points.append(np.array(self.climber.state[:2]))
    total = 0.0
    for i in range(len(points)-1):
      total += norm(points[i+1] - points[i])
    return total + self.slack

  def calculate_distance(self):
    if self.bolts:
      bottom = self.bolts[0].pos()
      top = self.bolts[-1].pos()
      return (norm(np.array(self.belayer.state[:2]) - bottom)
              + norm(top - bottom)
              + norm(np.array(self.climber.state[:2]) - top))
    points = [np.array(self.belayer.state[:2])]
    for bolt in self.bolts:
      points.append(np.array(bolt.pos()))
    points.append(np.array(self.climber.state[:2]))
    total = 0.0
    for i in range(len(points)-1):
      total += norm(points[i+1] - points[i])
    return total

  def stretch(self):
    return max(0.0,self.calculate_distance() - self.l0)

  def elastic_force(self):
    k1, k3 = self.k1, self.k3
    m = self.belayer.mass + self.climber.mass
    s = self.stretch()
    return -k1*s - k3*s**3