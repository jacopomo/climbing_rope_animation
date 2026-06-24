#!/usr/bin/env python3
from tkinter import *
import numpy as np
import time
from scipy.integrate import odeint
from scipy.optimize import fsolve
from numpy.linalg import norm

# Global Variable
RunMotion=False

# Canvas Data
ButtWidth=9
cw,ch=800,640
Ox,Oy=cw//2,ch//2
ar = ch/cw

# Physics Parameters
m = 80              # mass in kg
w1 = 8.6            # s^-1
w3 = 4.5            # m^-2*s^-1
delta = 8.6         # s^-1
k1 = m * w1**2      # N * m^-1
k3 = m * w3**2      # N * m^-3
L = 2.3             # slack in m
inclination = 15    # inclination in deg

# Parameters
g = 9.8
dt = 0.01
# Display
prad=2                  # pivot radius
rad=15                  # bob radius
bColor='red'            # bob color
scale=100.0             # pixels/m
tau=20                  # milliseconds

class Wall:
  def __init__(self, inclination):
      self.inclination = inclination

  def point_on(self, distance):
      '''
      distance in meters returns [x,y] in meters along the wall direction from pivot
      '''
      theta = np.radians(self.inclination)
      dx = distance * np.sin(theta)
      dy = distance * np.cos(theta)
      return dx, dy
  
  def normal_vector(self):
      theta = np.radians(self.inclination)
      return np.array([np.cos(theta), -np.sin(theta)])

  def distance(self, pos):
      return np.dot(np.array(pos), self.normal_vector())
  
  def draw(self):
      global cw, ch, scale
      d = np.sqrt((cw/scale)**2 + (ch/scale)**2)
      x1, y1 = meter2pix(self.point_on(-d))
      x2, y2 = meter2pix(self.point_on(d))
      normal = self.normal_vector()
      dx = -normal[0] * d * scale
      dy = normal[1] * d * scale
      x3, y3 = x2 + dx, y2 + dy
      x4, y4 = x1 + dx, y1 + dy
      canvas.create_polygon(x1, y1, x2, y2, x3, y3, x4, y4,
                            fill='#8B4513', stipple='gray25', outline='black')
      canvas.create_line(x1, y1, x2, y2, fill='black', width=2)

class Climber:
  def __init__(self, state, rad, grabbed=False):
      self.state = state
      self.rad = rad
      self.grabbed = grabbed
  
  def initialize_on_wall(self, wall):
      x = wall.point_on(L)[0] + wall.normal_vector()[0] * (1.01 * self.rad / scale)
      y = wall.point_on(L)[1] + wall.normal_vector()[1] * (1.01 * self.rad / scale)
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

  def release(self, event):
    self.state[2:]=[0.0,0.0]
    self.grabbed=False

  def move(self, wall):
    global BandImg, BobImg, nIter, Lab, t

    # integrate one step
    nIter+=1
    psoln=odeint(dfdt,self.state,t)
    self.state=psoln[1]

    # check for collision with wall
    self.collision(wall)

    # update graphics (circle)
    canvas.coords(BandImg,catenary(self.state[:2]))
    canvas.coords(BobImg,circ(self.state[:2]))
    if nIter%20==0:
      Lab[ITER]['text']=f'{nIter:d}'

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

# Start/Stop
def StartStop():
  global RunMotion
  RunMotion=not RunMotion
  StartButton['text']='Stop' if RunMotion else 'Restart'
  for ee in [ExitButton]+VarEntry:
    ee['state']=DISABLED if RunMotion else NORMAL

# Read Entries      
def ReadData(key):
  global dt,eta,inputs,L,Lab,k,m,scale,t,tau,VarEntry
  try:
    inputs[key]=float(VarEntry[key].get())
  except ValueError:
    return
  VarEntry[key].delete(0,END)
  VarEntry[key].insert(0,f'{inputs[key]:.3f}')
  Jacopo.state = inputs[:4]
  L,k,m,eta,scale,dt,tau=inputs[4:]
  tau=int(tau)
  t=[0.0,dt]

# Convert meters to pixels
def meter2pix(pos):
  global Ox,Oy,scale
  CanvPos=[]
  for i,xy in enumerate(pos):
    CanvPos.append(Ox+scale*xy if i%2==0 else Oy-scale*xy)
  return CanvPos

# Make a circle with radius rad centered at pos
def circ(pos):
  global rad
  cx,cy=meter2pix(pos)
  return [cx+rad,cy+rad,cx-rad,cy-rad]

# Function for a catenary curve
def CateFun(x,CatePar):
  L,cx,cy=CatePar
  rr=np.sqrt(L**2-cy**2)/cx
  return rr-np.sinh(x)/x

# Catenary function to compute the points of the catenary curve
def catenary(pos):
  global L,scale
  r=norm(pos)
  band=[Ox,Oy]
  if r<L:
    if abs(pos[0]*scale)<4:
      band.extend(meter2pix([0.5*pos[0],0.5*(pos[1]-L)]))
    else:
      absx=abs(pos[0])
      CatePar=[L,absx,pos[1]]
      AA0=0.01
      AA=fsolve(CateFun,AA0,CatePar)[0]
      aa=0.5*absx/AA
      bb=0.5*absx-aa*np.arctanh(pos[1]/L)
      cc=0.5*(pos[1]-L/np.tanh(AA))
      for i in range(1,20):
        x1=pos[0]*i/20.0
        band.extend(meter2pix([x1,aa*np.cosh((abs(x1)-bb)/aa)+cc]))
  band.extend(meter2pix(pos))
  return band

# Heaviside step function
def Phi(z):
    return np.where(z > 0, 1.0, 0.0)

# System of first-order ODEs describing the rope dynamics
def dfdt(state, t):
    global g, k1, k3, m, delta, scale
    pos = np.array(state[:2])
    vel = np.array(state[2:])
    r = norm(pos)
    stretch = r - L
    
    # Radial velocity component
    r_dot = np.dot(pos, vel) / r if r > 1e-6 else 0.0
    
    # Spring forces (only when stretched)
    if stretch > 0:
        force = -k1*stretch - k3*stretch**3
        force_vec = force * pos / r
        
        # Velocity damping (only when stretching)
        damping_force = -2*m*delta*r_dot * Phi(r_dot)
        damping_vec = damping_force * pos / r
        
        fx = force_vec[0] + damping_vec[0]
        fy = force_vec[1] + damping_vec[1] - m*g
    else:
        fx = 0
        fy = -m*g

    ax = fx / m
    ay = fy / m
    return [state[2], state[3], ax, ay]


# Initializations
rock = Wall(inclination)
Jacopo  = Climber([], rad)
Jacopo.initialize_on_wall(rock)

# Create Root window
root=Tk()
root.title('Catenary Pendulum')

# Add canvas to root window
canvas=Canvas(root,width=cw,height=ch,background='#ffffff')
canvas.grid(row=0,column=0)
canvas.bind('<Button-1>',Jacopo.grab)
canvas.bind('<B1-Motion>',Jacopo.drag)
canvas.bind('<ButtonRelease-1>',Jacopo.release)

# Add toolbar to root window
toolbar=Frame(root)
toolbar.grid(row=0,column=1,sticky=N)
toolbar.option_add('*Font','Helvetica 11')

# Toolbar buttons
nr=0
StartButton=Button(toolbar,text='Start',command=StartStop,\
  width=ButtWidth)
StartButton.grid(row=nr,column=0,sticky=W)
nr+=1
ExitButton=Button(toolbar,text='Exit',command=quit,width=ButtWidth)
ExitButton.grid(row=nr,column=0,sticky=W)
nr+=1

# Label and Entry arrays
VarLab=['x\u2080 (m)','y\u2080 (m)','vx\u2080 (m/s)','vy\u2080 (m/s)',\
  'Length (m)','k1 (N/m)','k3 (N/m³)','Mass (kg)','\u03B7 (Ns/m)','scale (px/m)',\
    'Time step (s)','\u03C4 (ms)']
inputs=Jacopo.state+[L,k1,k3,m,delta,scale,dt,tau]
VarEntry=[]
for i,lab in enumerate(VarLab):
  Label(toolbar,text=str(lab)).grid(row=nr,column=0)
  VarEntry.append(Entry(toolbar,bd=5,width=ButtWidth))
  VarEntry[i].grid(row=nr,column=1)
  VarEntry[i].insert(0,f'{inputs[i]:.3f}')
  VarEntry[i].bind('<Return>',lambda event,n=i:ReadData(n))
  nr+=1

# Labels
LabList=['Period','Iterations']
PERIOD,ITER=range(2)
Lab=[]
for i,ll in enumerate(LabList):
  Label(toolbar,text=ll,).grid(row=nr,column=0)
  Lab.append(Label(toolbar,text='     '))
  Lab[i].grid(row=nr,column=1,sticky=W)
  nr+=1

# Draw Pivot, Band, Bob, and wall
canvas.create_oval(Ox-prad,Oy-prad,Ox+prad,Oy+prad,fill='black')
BandImg=canvas.create_line([Ox,Oy]+meter2pix(Jacopo.state[:2]),fill='black')
BobImg=canvas.create_oval(circ(Jacopo.state[:2]),fill=bColor)
rock.draw()

# ...................................................................
t=[0.0,dt]
tcount=0
nIter=0
tt0=time.time()

# Main animation loop
def animate():
  global Lab,tcount,tt0
  StartIter=time.time()
  if RunMotion:
    Jacopo.move(wall=rock)
  # Cycle Duration
  tcount+=1
  if tcount%20==0:
    ttt=time.time()
    Lab[PERIOD]['text']=f'{(ttt-tt0)*50.0:8.3}'+' ms'
    tt0=ttt
  # .................................................................
  ElapsIter=int((time.time()-StartIter)*1000.0)
  canvas.after(max(1,tau-ElapsIter),animate)
  #------------------------------------------------------------------
animate()
root.mainloop()