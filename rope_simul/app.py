import time
from tkinter import *
import numpy as np
from scipy.integrate import odeint
from scipy.optimize import fsolve

from . import config
from .utils import meter2pix, circ


class RopeApp:
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
        self._draw_static_objects()
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

    def _draw_static_objects(self):
        Ox, Oy = config.Ox, config.Oy
        prad = config.prad
        self.canvas.create_oval(Ox-prad, Oy-prad, Ox+prad, Oy+prad, fill='black')
        self.wall.draw(self.canvas)
        self.band_id = self.canvas.create_line([Ox, Oy] + meter2pix(self.climber.state[:2]), fill='black')
        self.bob_id = self.canvas.create_oval(circ(self.climber.state[:2], self.climber.rad), fill=config.bColor)

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

        self.climber.state = self.inputs[:4]
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

        self.climber.state = self.inputs[:4]
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
        self.update_graphics()

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
        pos = np.array(state[:2])
        vel = np.array(state[2:])
        r = np.linalg.norm(pos)
        stretch = self.rope.stretch()
        m = self.climber.mass

        r_dot = np.dot(pos, vel) / r if r > 1e-6 else 0.0
        if stretch > 0:
            force = self.rope.elastic_force()
            force_vec = force * pos / r
            damping_force = -2 * m * config.delta * r_dot * (1.0 if r_dot > 0 else 0.0)
            damping_vec = damping_force * pos / r
            fx = force_vec[0] + damping_vec[0]
            fy = force_vec[1] + damping_vec[1] - m * config.g
        else:
            fx = 0.0
            fy = -m * config.g

        return [vel[0], vel[1], fx / m, fy / m]

    def step(self):
        self.n_iter += 1
        solution = odeint(self._dfdt, self.climber.state, self.t)
        self.climber.state = solution[1].tolist()
        self.climber.collision(self.wall)
        if self.n_iter % 20 == 0:
            self.iter_label.config(text=f'{self.n_iter:d}')
        self.update_graphics()

    def update_graphics(self):
        self.canvas.coords(self.band_id, [config.Ox, config.Oy] + self.catenary(self.climber.state[:2]))
        self.canvas.coords(self.bob_id, circ(self.climber.state[:2], self.climber.rad))

    def catenary(self, pos):
        r = np.linalg.norm(pos)
        band = [config.Ox, config.Oy]
        if r < self.rope.slack:
            if abs(pos[0] * config.scale) < 4:
                band.extend(meter2pix([0.5 * pos[0], 0.5 * (pos[1] - self.rope.slack)]))
            else:
                absx = abs(pos[0])
                cate_par = [self.rope.slack, absx, pos[1]]
                AA0 = 0.01
                AA = fsolve(self._cate_fun, AA0, cate_par)[0]
                aa = 0.5 * absx / AA
                bb = 0.5 * absx - aa * np.arctanh(pos[1] / self.rope.slack)
                cc = 0.5 * (pos[1] - self.rope.slack / np.tanh(AA))
                for i in range(1, 20):
                    x1 = pos[0] * i / 20.0
                    band.extend(meter2pix([x1, aa * np.cosh((abs(x1) - bb) / aa) + cc]))
        band.extend(meter2pix(pos))
        return band

    def _cate_fun(self, x, cate_par):
        L, cx, cy = cate_par
        rr = np.sqrt(L**2 - cy**2) / cx
        return rr - np.sinh(x) / x

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

