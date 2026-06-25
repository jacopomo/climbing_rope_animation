#!/usr/bin/env python3
from rope_simul import Belayer, Climber, Rope, Wall, RopeApp
import rope_simul.config as config

rock = Wall(config.inclination)
Jacopo = Climber([], config.rad, config.m)
Nico = Belayer([0,0,0,0], config.rad, config.m)
Edelrid = Rope([config.k1, config.k3], Nico, Jacopo, config.L)
Jacopo.initialize_on_wall(rock, dist=config.L)

RopeApp(Nico, Jacopo, Edelrid, rock).run()
