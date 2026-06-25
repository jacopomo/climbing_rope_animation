#!/usr/bin/env python3
from rope_simul import Person, Rope, Wall, Simulation
import rope_simul.config as config

rock = Wall(config.inclination)
belayer = Person([], config.rad, config.m)
climber = Person([], config.rad, config.m)
climber.initialize_on_wall(rock, dist=config.L)
belayer.initialize_on_floor(rock)
Edelrid = Rope([config.k1, config.k3], belayer, climber, config.L)

Simulation(belayer, climber, Edelrid, rock).run()
