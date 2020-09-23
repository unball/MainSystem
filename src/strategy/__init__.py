from .entity.attacker import Attacker
import numpy as np
import time

class Strategy:
    def __init__(self, world):
        self.world = world

    def entityDecider(self):
        robot = self.world.team[0]
        robot.updateEntity(Attacker)
        # robot = self.world.team[1]
        # robot.updateEntity(Attacker)
        # robot = self.world.team[2]
        # robot.updateEntity(Attacker)
    
    def update(self):
        self.entityDecider()
        for robot in self.world.team:
            if robot.entity is not None:
                robot.entity.directionDecider()
                robot.entity.fieldDecider()

