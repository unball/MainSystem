from .entity.attacker import Attacker
from .entity.goalKeeper import GoalKeeper
from .entity.defender import Defender
import numpy as np
import time

class Strategy:
    def __init__(self, world):
        self.world = world

    def entityDecider(self):
        self.world.team[0].updateEntity(Attacker)
        self.world.team[1].updateEntity(GoalKeeper)
        self.world.team[2].updateEntity(Defender)
    
    def update(self):
        self.entityDecider()
        for robot in self.world.team:
            if robot.entity is not None:
                robot.updateSpin()
                robot.entity.directionDecider()
                robot.entity.fieldDecider()
