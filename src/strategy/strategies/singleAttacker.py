from strategy.entity.attacker import Attacker
from strategy import Strategy
import numpy as np
import time

class SingleAttacker(Strategy):
    def __init__(self, world, params):
        Strategy.__init__(self, world)

        self.params = params

    def entityDecider(self):
        robot = self.world.team[0]
        robot.updateEntity(Attacker, **self.params)