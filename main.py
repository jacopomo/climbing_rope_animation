#!/usr/bin/env python3
from rope_simul import Person, Bolt, Rope, Wall, Simulation
import rope_simul.config as config

rock = Wall(config.inclination)
bolts = Bolt.generate(rock, N=5)
bottom_bolt, top_bolt = bolts[0], bolts[-1]

rad, mass = config.rad, config.m
belayer = Person(rad=rad, mass=mass, bolt=bottom_bolt)
climber = Person(rad=rad, mass=mass, bolt=top_bolt)

belayer.initialize_on_floor()
climber.initialize_on_wall(rock, dist=1.0)

rope_specs = (config.k1, config.k3)
rope = Rope(rope_specs, belayer, climber, bolts=bolts, slack=0.0)

Simulation(belayer, climber, rope, rock).run()
