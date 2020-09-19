from strategy.field import UVF
from strategy.movements import goToBall
from tools import angError
from tools.interval import Interval
import numpy as np
import time

class Strategy:
    def __init__(self, world):
        self.world = world
        self.lastDirectionChange = 0
    
    def update(self):
        robot = self.world.team[0]
        rb = np.array(self.world.ball.pos)
        rg = np.array((0.65, 0))

        if robot.field is not None:
            ref_th = robot.field.F(robot.pose)
            rob_th = robot.th

            if angError(ref_th, rob_th) > 120 * np.pi / 180:
                robot.direction *= -1

        Pb = goToBall(rb, rg)

        robot.field = UVF(Pb)