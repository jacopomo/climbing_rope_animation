import time
from tkinter import *
import numpy as np
from scipy.integrate import odeint

from . import config
from .utils import meter2pix, circ


class Simulation:
    def __init__(self, belayer, climber, rope, wall):
        self.belayer = belayer
        self.climber = climber
        self.rope = rope
        self.wall = wall

        self.run_motion = False
        self.n_iter = 0
        self.tcount = 0
        self.tt0 = time.time()
        self.t = [0.0, config.dt]

        self.inputs = self.climber.state + [
            self.rope.slack,
            self.rope.k1,
            self.rope.k3,
            self.climber.mass,
            config.delta,
            config.scale,
            config.dt,
            config.tau,
        ]
        self.initial_inputs = self.inputs.copy()

        self.root = Tk()
        self.root.title('Climbing Fall Simulation')
        self._build_ui()
        self._init_canvas_items()
        self.update_graphics()

    def _build_ui(self):
        cw, ch = config.cw, config.ch

        self.canvas = Canvas(self.root, width=cw, height=ch, background='#ffffff')
        self.canvas.grid(row=0, column=0)
        self.canvas.bind('<Button-1>', self.on_grab)
        self.canvas.bind('<B1-Motion>', self.on_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_release)

        toolbar = Frame(self.root)
        toolbar.grid(row=0, column=1, sticky=N)
        toolbar.option_add('*Font', 'Helvetica 11')

        self.start_button = Button(toolbar, text='Start', command=self.start_stop, width=config.ButtWidth)
        self.start_button.grid(row=0, column=0, sticky=W)
        self.restart_button = Button(toolbar, text='Restart', command=self.restart, width=config.ButtWidth)
        self.restart_button.grid(row=0, column=1, sticky=W)
        self.exit_button = Button(toolbar, text='Exit', command=self.root.quit, width=config.ButtWidth)
        self.exit_button.grid(row=1, column=0, sticky=W)

        self.var_labels = [
            'x₀ (m)',
            'y₀ (m)',
            'vx₀ (m/s)',
            'vy₀ (m/s)',
            'Length (m)',
            'k1 (N/m)',
            'k3 (N/m³)',
            'Mass (kg)',
            'δ (Ns/m)',
            'scale (px/m)',
            'Time step (s)',
            'τ (ms)',
        ]

        self.var_entries = []
        row = 2
        for i, label_text in enumerate(self.var_labels):
            Label(toolbar, text=label_text).grid(row=row, column=0)
            entry = Entry(toolbar, bd=5, width=config.ButtWidth)
            entry.grid(row=row, column=1)
            entry.insert(0, f'{self.inputs[i]:.3f}')
            entry.bind('<Return>', lambda event, n=i: self.read_data(n))
            self.var_entries.append(entry)
            row += 1

        self.period_label = Label(toolbar, text='     ')
        Label(toolbar, text='Period').grid(row=row, column=0)
        self.period_label.grid(row=row, column=1, sticky=W)
        row += 1

        self.iter_label = Label(toolbar, text='     ')
        Label(toolbar, text='Iterations').grid(row=row, column=0)
        self.iter_label.grid(row=row, column=1, sticky=W)

    def _init_canvas_items(self):
        bolt_rad = config.bolt_rad
        self.wall.draw(self.canvas)

        # draw floor as horizontal line
        floor_y_px = meter2pix([0.0, (config.Oy - config.ch + 0) / config.scale])[1]
        self.canvas.create_line(0, floor_y_px, config.cw, floor_y_px, fill='black', width=4)

        for bolt in self.rope.bolts:
            bolt.draw(self.canvas, meter2pix, bolt_rad)

        band_coords = [coord for point in self._rope_points() for coord in meter2pix(point)]
        self.band_id = self.canvas.create_line(band_coords, fill='black')
        self.belayer_id = self.canvas.create_oval(circ(self.belayer.state[:2], self.belayer.rad), fill=config.bColor)
        self.climber_id = self.canvas.create_oval(circ(self.climber.state[:2], self.climber.rad), fill=config.cColor)

    def start_stop(self):
        self.run_motion = not self.run_motion
        self.start_button.config(text='Stop' if self.run_motion else 'Start')
        state = DISABLED if self.run_motion else NORMAL
        for widget in [self.exit_button, self.restart_button] + self.var_entries:
            widget['state'] = state

    def restart(self):
        self.run_motion = False
        self.start_button.config(text='Start')

        self.inputs = self.initial_inputs.copy()
        for i, entry in enumerate(self.var_entries):
            entry['state'] = NORMAL
            entry.delete(0, END)
            entry.insert(0, f'{self.inputs[i]:.3f}')

        self._apply_inputs()

        self.n_iter = 0
        self.tcount = 0
        self.tt0 = time.time()
        self.iter_label.config(text=f'{self.n_iter:d}')
        self.update_graphics()

    def read_data(self, key):
        try:
            value = float(self.var_entries[key].get())
        except ValueError:
            return

        self.inputs[key] = value
        self.var_entries[key].delete(0, END)
        self.var_entries[key].insert(0, f'{value:.3f}')

        self._apply_inputs()
        self.update_graphics()

    def _apply_inputs(self):
        self.climber.state = self.inputs[:4].copy()
        self.rope.slack = self.inputs[4]
        self.rope.k1 = self.inputs[5]
        self.rope.k3 = self.inputs[6]
        self.climber.mass = self.inputs[7]
        config.delta = self.inputs[8]
        config.scale = self.inputs[9]
        config.dt = self.inputs[10]
        config.tau = int(self.inputs[11])
        self.t = [0.0, config.dt]
        self.rope.l0 = self.rope.initial_length()

    def on_grab(self, event):
        if self.run_motion:
            return
        bob_x, bob_y = meter2pix(self.climber.state[:2])
        if np.hypot(event.x - bob_x, event.y - bob_y) < self.climber.rad:
            self.climber.grabbed = True

    def on_drag(self, event):
        if not self.climber.grabbed:
            return
        x = (np.clip(event.x, self.climber.rad, config.cw - self.climber.rad) - config.Ox) / config.scale
        y = (config.Oy - np.clip(event.y, self.climber.rad, config.ch - self.climber.rad)) / config.scale
        self.climber.state[0:2] = [x, y]
        self.update_graphics()

    def on_release(self, event):
        if not self.climber.grabbed:
            return
        self.climber.state[2:] = [0.0, 0.0]
        self.climber.grabbed = False
        self.update_graphics()

    def _dfdt(self, state, t):
        # state is [b_x, b_y, b_vx, b_vy, c_x, c_y, c_vx, c_vy]
        # positions and velocities for belayer
        pos_b = np.array(state[0:2])
        vel_b = np.array(state[2:4])
        # positions and velocities for climber
        pos_c = np.array(state[4:6])
        vel_c = np.array(state[6:8])

        # compute segment vectors for belayer->first_bolt and last_bolt->climber
        if hasattr(self.rope, 'bolts') and self.rope.bolts:
            first_pos = np.array(self.rope.bolts[0].pos())
            last_pos = np.array(self.rope.bolts[-1].pos())
        else:
            # default to origin for single-pulley setup
            first_pos = np.array([0.0, 0.0])
            last_pos = np.array([0.0, 0.0])

        dir_b = first_pos - pos_b
        dir_c = last_pos - pos_c
        r_b = np.linalg.norm(dir_b)
        r_c = np.linalg.norm(dir_c)

        # total stretch based on piecewise distances
        s = max(0.0, self.rope.calculate_distance() - self.rope.l0)

        # rope tension magnitude (positive) - rope.elastic_force() returns negative when stretched
        Tmag = -self.rope.elastic_force() if s > 0.0 else 0.0

        # radial velocities (projection of velocity onto segment direction)
        rdot_b = np.dot(dir_b, vel_b) / r_b if r_b > 1e-6 else 0.0
        rdot_c = np.dot(dir_c, vel_c) / r_c if r_c > 1e-6 else 0.0

        # simple damping along the rope segments (only when extending)
        damp_b = -config.delta * rdot_b * (1.0 if rdot_b > 0 else 0.0)
        damp_c = -config.delta * rdot_c * (1.0 if rdot_c > 0 else 0.0)

        m_b = self.belayer.mass
        m_c = self.climber.mass

        # friction through all bolts: linear in rope sliding speed (scaled by number of bolts)
        n_bolts = len(self.rope.bolts) if hasattr(self.rope, 'bolts') else 0
        gamma = getattr(config, 'bolt_friction', 0.0) * max(1, n_bolts)
        # sliding speed approximation
        v_slide = rdot_b + rdot_c
        Ff = -gamma * v_slide

        # unit direction vectors
        ub = dir_b / r_b if r_b > 1e-6 else np.array([0.0, 0.0])
        uc = dir_c / r_c if r_c > 1e-6 else np.array([0.0, 0.0])

        # forces on belayer: tension along its segment toward the bolt + damping + bolt friction, minus gravity
        if r_b > 1e-6 and Tmag != 0.0:
            force_b_vec = Tmag * ub
            damping_b_vec = damp_b * ub
            bolt_b_vec = Ff * ub
            fx_b = force_b_vec[0] + damping_b_vec[0] + bolt_b_vec[0]
            fy_b = force_b_vec[1] + damping_b_vec[1] + bolt_b_vec[1] - m_b * config.g
        else:
            fx_b = 0.0
            fy_b = -m_b * config.g
        # forces on climber: tension along its segment toward the bolt + damping + bolt friction, minus gravity
        if r_c > 1e-6 and Tmag != 0.0:
            force_c_vec = Tmag * uc
            damping_c_vec = damp_c * uc
            bolt_c_vec = -Ff * uc
            fx_c = force_c_vec[0] + damping_c_vec[0] + bolt_c_vec[0]
            fy_c = force_c_vec[1] + damping_c_vec[1] + bolt_c_vec[1] - m_c * config.g
        else:
            fx_c = 0.0
            fy_c = -m_c * config.g

        # accelerations
        ax_b = fx_b / m_b
        ay_b = fy_b / m_b
        ax_c = fx_c / m_c
        ay_c = fy_c / m_c

        return [vel_b[0], vel_b[1], ax_b, ay_b, vel_c[0], vel_c[1], ax_c, ay_c]

    def step(self):
        self.n_iter += 1
        # integrate both belayer and climber states together
        combined = self.belayer.state + self.climber.state
        solution = odeint(self._dfdt, combined, self.t)
        new = solution[1].tolist()
        self.belayer.state = new[0:4]
        self.climber.state = new[4:8]
        # handle collisions
        self.climber.collision(self.wall)
        self.belayer.collision(self.wall)
        if self.n_iter % 20 == 0:
            self.iter_label.config(text=f'{self.n_iter:d}')
        self.update_graphics()

    def _rope_points(self):
        points = [self.belayer.state[:2]]
        for bolt in self.rope.bolts:
            points.append(bolt.pos().tolist())
        points.append(self.climber.state[:2])
        return points

    def update_graphics(self):
        band_coords = [coord for point in self._rope_points() for coord in meter2pix(point)]
        self.canvas.coords(self.band_id, band_coords)
        self.canvas.coords(self.belayer_id, circ(self.belayer.state[:2], self.belayer.rad))
        self.canvas.coords(self.climber_id, circ(self.climber.state[:2], self.climber.rad))
    def animate(self):
        start_iter = time.time()
        if self.run_motion:
            self.step()

        self.tcount += 1
        if self.tcount % 20 == 0:
            now = time.time()
            self.period_label.config(text=f'{(now - self.tt0) * 50.0:8.3} ms')
            self.tt0 = now

        elapsed = int((time.time() - start_iter) * 1000.0)
        self.root.after(max(1, config.tau - elapsed), self.animate)

    def run(self):
        self.animate()
        self.root.mainloop()

