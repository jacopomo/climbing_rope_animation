#!/usr/bin/env python3
from tkinter import *
import numpy as np
import time
from scipy.integrate import odeint
from scipy.optimize import fsolve
from numpy.linalg import norm

# Global Variables
Grabbed=RunMotion=False

# Canvas Data
ButtWidth=9
cw,ch=800,640
Ox,Oy=cw//2,ch//2

# Physics Parameters
m = 80              # mass in kg
w1 = 8.6            # s^-1
w3 = 4.5            # m^-2*s^-1
delta = 8.6         # s^-1
k1 = m * w1**2      # N * m^-1
k3 = m * w3**2      # N * m^-3
L = 2.3             # slack in m

# Parameters
g = 9.8
dt = 0.01
# Display
prad=2                  # pivot radius
rad=6                   # bob radius
bColor='red'            # bob color
scale=100.0             # pixels/m
tau=20                  # milliseconds

# Initial position and velocity
state=[0.0,2.3,0,0.0]   # [x0,y0,vx0,vy0]

# Start/Stop
def StartStop():
  global RunMotion
  RunMotion=not RunMotion
  StartButton['text']='Stop' if RunMotion else 'Restart'
  for ee in [ExitButton]+VarEntry:
    ee['state']=DISABLED if RunMotion else NORMAL

# Read Entries      
def ReadData(key):
  global dt,eta,inputs,L,Lab,k,m,scale,state,t,tau,VarEntry
  try:
    inputs[key]=float(VarEntry[key].get())
  except ValueError:
    return
  VarEntry[key].delete(0,END)
  VarEntry[key].insert(0,f'{inputs[key]:.3f}')
  state=inputs[:4]
  L,k,m,eta,scale,dt,tau=inputs[4:]
  tau=int(tau)
  t=[0.0,dt]
  canvas.coords(circle,meter2pix([-L,L,L,-L]))

# Grab ball
def GrabBall(event):
  global Grabbed,rad,RunMotion,state
  if not RunMotion:
    dist=np.array(meter2pix(state[:2]))-np.array([event.x,event.y])
    Grabbed=norm(dist)<rad

# Drag ball
def DragBall(event):
  global Grabbed,Ox,Oy,rad,scale,state
  if Grabbed:
    state[0]=(np.clip(event.x,rad,cw-rad)-Ox)/scale
    state[1]=(Oy-np.clip(event.y,rad,ch-rad))/scale

# Release ball
def ReleaseBall(event):
  global Grabbed,Lab,state
  state[2:]=[0.0,0.0]
  Grabbed=False

# Convert meters to pixels
def meter2pix(pos):
  global Ox,Oy,scale
  CanvPos=[]
  for i,xy in enumerate(pos):
    CanvPos.append(Ox+scale*xy if i%2==0 else Oy-scale*xy)
  return CanvPos

#  Make a circle with radius rad centered at pos
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
def dfdt(state,t):
    global g,k1,k3,m,delta
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

# Create Root window
root=Tk()
root.title('Catenary Pendulum')

# Add canvas to root window
canvas=Canvas(root,width=cw,height=ch,background='#ffffff')
canvas.grid(row=0,column=0)
canvas.bind('<Button-1>',GrabBall)
canvas.bind('<B1-Motion>',DragBall)
canvas.bind('<ButtonRelease-1>',ReleaseBall)

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
inputs=state+[L,k1,k3,m,delta,scale,dt,tau]
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

# Draw Circle and Horizontal Line
circle=canvas.create_oval(meter2pix([-L,L,L,-L]),outline='green')
canvas.create_line(0,ch-Oy,cw,ch-Oy,fill='green')

# Draw Pendulum
canvas.create_oval(Ox-prad,Oy-prad,Ox+prad,Oy+prad,fill='black')
BandImg=canvas.create_line([Ox,Oy]+meter2pix(state[:2]),fill='black')
BobImg=canvas.create_oval(circ(state[:2]),fill=bColor)

# ...................................................................
t=[0.0,dt]
tcount=0
nIter=0
tt0=time.time()

# Main animation loop
def animate():
  global BandImg,BobImg,Lab,nIter,state,t,tcount,tt0
  StartIter=time.time()
  # Draw pendulum
  canvas.coords(BandImg,catenary(state[:2]))
  canvas.coords(BobImg,circ(state[:2]))
  if RunMotion:
    nIter+=1
    # Velocity and position for next frame
    psoln=odeint(dfdt,state,t)
    state=psoln[1]
    if nIter%20==0:
      Lab[ITER]['text']=f'{nIter:d}'

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