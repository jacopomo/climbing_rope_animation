# from .utils import SOMETHING
# from .conig import SOMETHING ELSE
import numpy as np
from numpy.linalg import norm


class Rope:
  def __init__(self, specifics, belayer, climber, slack=0.0):
    self.k1, self.k3 = specifics
    self.belayer = belayer
    self.climber = climber
    self.slack = slack
    # l0 is how much total rope is in the system
    self.l0 = self.initial_length()

  def initial_length(self):
    # Belayer to bolt distance:
    b_dist = norm(np.array(self.belayer.state[:2]))
    # Climber to bolt distance:
    c_dist = norm(np.array(self.climber.state[:2]))
    # Total rope length is the sum plus slack
    return b_dist + c_dist + self.slack

  def calculate_distance(self):
    # Belayer to bolt distance:
    b_dist = norm(np.array(self.belayer.state[:2]))
    # Climber to bolt distance:
    c_dist = norm(np.array(self.climber.state[:2]))
    return b_dist + c_dist 

  def stretch(self):
    return max(0.0,self.calculate_distance() - self.l0)

  def elastic_force(self):
    k1, k3 = self.k1, self.k3
    m = self.belayer.mass + self.climber.mass
    s = self.stretch()
    return -k1*s - k3*s**3