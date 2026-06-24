"""
Global Configuration and Shared State
"""

RunMotion = False

L = 2.3                 # Slack in meters
inclination = 15        # Inclination in degrees

ButtWidth = 9
cw, ch = 800, 640
Ox, Oy = cw // 2, ch // 2

prad = 2                # Pivot radius
rad = 15                # Bob radius
bColor = 'red'          # Bob color
scale = 100.0           # Pixels per meter
tau = 20                # Timer interval in milliseconds
dt = 0.01               # Physics time step

m = 80                  # Mass in kg
w1 = 8.6                # s^-1
w3 = 4.5                # m^-2 * s^-1
delta = 8.6             # s^-1
g = 9.8

k1 = m * w1**2          # N * m^-1
k3 = m * w3**2          # N * m^-3