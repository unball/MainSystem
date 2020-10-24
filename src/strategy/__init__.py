from .entity.attacker import Attacker
import numpy as np
import time

class Strategy:
    def __init__(self, world):
        self.world = world

    def entityDecider(self):
        self.world.team[0].updateEntity(Attacker)
        self.world.team[1].updateEntity(Attacker)
        # robot = self.world.team[2]
        # robot.updateEntity(Attacker)
        
        # robot = self.world.enemies[0]
        # robot.updateEntity(Attacker,side=-1)
        # robot = self.world.enemies[1]
        # robot.updateEntity(Attacker,side=-1)
        # robot = self.world.enemies[2]
        # robot.updateEntity(Attacker,side=-1)
    
    def update(self):
        self.entityDecider()
        for robot in self.world.team:
            if robot.entity is not None:
                robot.entity.directionDecider()
                robot.entity.fieldDecider()

        # for robot in self.world.enemies:
        #     if robot.entity is not None:
        #         robot.entity.directionDecider()
        #         robot.entity.fieldDecider()
