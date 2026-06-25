#!/usr/bin/env python3
from rope_simul import Wall, Simulation, Bolt
import rope_simul.config as config

rock = Wall(config.inclination)

# create scene (bolts, belayer, climber, rope) with a single call
belayer, climber, Edelrid, bolts = Bolt.create_scene(rock, N=5)

Simulation(belayer, climber, Edelrid, rock).run()
