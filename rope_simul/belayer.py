import numpy as np

class Belayer:
  def __init__(self, state, rad, mass, grabbed=False):
      self.state = state
      self.rad = rad
      self.mass = mass
      self.grabbed = grabbed