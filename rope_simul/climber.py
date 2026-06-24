import numpy as np

from . import config

class Climber:
  def __init__(self, state, rad, mass, grabbed=False):
      self.state = state
      self.rad = rad
      self.mass = mass
      self.grabbed = grabbed
  
  def initialize_on_wall(self, wall, dist):
      x = wall.point_on(dist)[0] + wall.normal_vector()[0] * (1.01 * self.rad / config.scale)
      y = wall.point_on(dist)[1] + wall.normal_vector()[1] * (1.01 * self.rad / config.scale)
      self.state = [x, y, 0.0, 0.0]
  
  def grab(self, event):
    global RunMotion
    if not RunMotion:
      dist=np.array(meter2pix(self.state[:2]))-np.array([event.x,event.y])
      self.grabbed=norm(dist)<self.rad
  
  def drag(self, event):
    if self.grabbed:
      self.state[0]=(np.clip(event.x,self.rad,cw-self.rad)-Ox)/scale
      self.state[1]=(Oy-np.clip(event.y,self.rad,ch-self.rad))/scale
      # update graphics immediately so the bob follows the mouse
      try:
        canvas.coords(BandImg,catenary(self.state[:2]))
        canvas.coords(BobImg,circ(self.state[:2]))
      except Exception:
        pass

  def release(self, event):
    self.state[2:]=[0.0,0.0]
    self.grabbed=False
    # ensure final position is shown
    try:
      canvas.coords(BandImg,catenary(self.state[:2]))
      canvas.coords(BobImg,circ(self.state[:2]))
    except Exception:
      pass

  def move(self, wall):
    BandImg, BobImg, Lab, t = config.BandImg, config.BobImg, config.Lab, config.t

    # integrate one step
    config.nIter+=1
    psoln=odeint(dfdt,self.state,t)
    self.state=psoln[1]

    # check for collision with wall
    self.collision(wall)

    # update graphics (circle)
    canvas.coords(BandImg,catenary(self.state[:2]))
    canvas.coords(BobImg,circ(self.state[:2]))
    if config.nIter%20==0:
      Lab[ITER]['text']=f'{config.nIter:d}'

  def collision(self, wall):
    pos = np.array(self.state[:2])
    vel = np.array(self.state[2:])
    d_signed = wall.distance(pos)
    radius_m = self.rad/scale
    penetration = radius_m - d_signed
    if penetration > 0.0:
      normal = wall.normal_vector()
      vel_n = np.dot(vel, normal)
      # set normal component to -e * vel_n (e=0.5)
      e = 0.25
      vel_t = vel - vel_n * normal
      vel_post = vel_t - e * vel_n * normal
      # update velocities
      self.state[2] = vel_post[0]
      self.state[3] = vel_post[1]
      # nudge position out of wall
      self.state[0] += normal[0] * (penetration + 1e-4)
      self.state[1] += normal[1] * (penetration + 1e-4)
