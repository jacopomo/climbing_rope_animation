"""
Global Configuration and Shared State
"""

L = 1.0                 # Slack in meters
inclination = 15        # Inclination in degrees

ButtWidth = 9
cw, ch = 1400, 900
Ox, Oy = cw // 2, ch // 2

bolt_rad = 4            # Bolt radius
rad = 15                # Bob radius
bColor = 'blue'         # Belayer color
cColor = 'red'          # Climber color

scale = 100.0           # Pixels per meter
tau = 20                # Timer interval in milliseconds
dt = 0.01               # Physics time step

m = 80                  # Mass in kg
w1 = 8.6                # s^-1
w3 = 4.5                # m^-2 * s^-1
delta = 8.6             # s^-1
g = 9.8

# friction coefficient at the bolt (N per (m/s) of rope sliding).
# A small positive value dissipates energy when the rope runs over the bolt.
bolt_friction = 0.0

k1 = m * w1**2          # N * m^-1
k3 = m * w3**2          # N * m^-3

x = y = vx = vy = 0.0
inputs = [x, y, vx, vy, L, k1, k3, m, delta, scale, dt, tau]