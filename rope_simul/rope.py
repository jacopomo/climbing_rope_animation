# from .utils import SOMETHING
# from .conig import SOMETHING ELSE
import numpy as np
from numpy.linalg import norm


class Rope:
  def __init__(self, specifics, belayer, climber, bolts, slack=0.0):
    self.k1, self.k3 = specifics
    self.belayer = belayer
    self.climber = climber
    self.bolts = bolts
    self.slack = slack

    # l0 is how much total rope is in the system at the start
    self.l0 = self.calculate_distance() + self.slack

  def calculate_distance(self):
    bottom = self.bolts[0].pos()
    top = self.bolts[-1].pos()
    return (norm(np.array(self.belayer.state[:2]) - bottom)
            + norm(top - bottom)
            + norm(np.array(self.climber.state[:2]) - top))
  
  def stretch(self):
    return max(0.0,self.calculate_distance() - self.l0)

  def elastic_force(self):
    k1, k3 = self.k1, self.k3
    s = self.stretch()
    return -k1*s - k3*s**3
  
  def elastic_energy(self):
    k1, k3 = self.k1, self.k3
    s = self.stretch()
    return 0.5*k1*s**2 + 0.25*k3*s**4