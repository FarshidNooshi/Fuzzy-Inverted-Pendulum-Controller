#!/usr/bin/env python
# -*- coding: utf-8 -*-

# project imports
from MyControllerFinal import MyFuzzyControllerFinal
from conf import ConfigReader
from myController import MyFuzzyController
from world import World
from manager import Manager


conf = ConfigReader()

if __name__ == '__main__':

    # world = World(**conf.world_config())
    # controller = MyFuzzyController()
    # manager = Manager(world, controller, **conf.simulation_config())
    # manager.run()

    world = World(**conf.world_config())
    controller = MyFuzzyControllerFinal()
    manager = Manager(world, controller, **conf.simulation_config())
    manager.run()