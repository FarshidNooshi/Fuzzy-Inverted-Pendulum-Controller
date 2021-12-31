#!/usr/bin/env python
# -*- coding: utf-8 -*-

# project imports
from conf import ConfigReader
from myController import MyFuzzyController
from world import World
from controller2 import FuzzyController2
from manager import Manager


conf = ConfigReader()

if __name__ == '__main__':
    # world = World(**conf.world_config())
    # controller = FuzzyController2()
    # manager = Manager(world, controller, **conf.simulation_config())
    # manager.run()

    world = World(**conf.world_config())
    controller = MyFuzzyController()
    manager = Manager(world, controller, **conf.simulation_config())
    manager.run()
