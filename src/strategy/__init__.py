from strategy.field import UVF, DirectionalField
from strategy.movements import goToBall
from tools import angError, howFrontBall, howPerpBall, ang, norml
from tools.interval import Interval
import numpy as np
import time

class Strategy:
    def __init__(self, world):
        self.world = world
        self.lastDirectionChange = 0
        self.attackAngle = None
        self.attackState = 0
        self.vravg = 0
    
    def update(self):
        robot = self.world.team[0]
        rr = np.array(robot.pos)
        vr = np.array(robot.v)
        self.vravg = 0.995 * self.vravg + 0.005 * norml(vr)
        rb = np.array(self.world.ball.pos)
        vb = np.array(self.world.ball.v)
        rg = np.array(self.world.field.goalPos)
        rl = np.array(self.world.field.marginPos)

        if robot.field is not None:
            ref_th = robot.field.F(robot.pose)
            rob_th = robot.th

            if abs(angError(ref_th, rob_th)) > 120 * np.pi / 180:
                robot.direction *= -1

        # Define estado do movimento
        if self.attackState == 0:
            if -howFrontBall(rb, rr, rg)  > 0 and abs(howPerpBall(rb, rr, rg)) < 0.075 * 0.5 and abs(angError(robot.th, ang(rb, rg))) < 20 * np.pi / 180:
                self.attackState = 1
                self.attackAngle = robot.th
                #print("Modo ataque!")
            else: self.attackState = 0
        else:
            if -howFrontBall(rb, rr, rg)  > 0 and abs(howPerpBall(rb, rr, rg)) < 0.075*2 and abs(angError(robot.th, ang(rb, rg))) < 90 * np.pi / 180:
                self.attackState = 1
            else:
                #print("Saiu modo ataque!")
                self.attackState = 0

        # Aplica o movimento
        if self.attackState == 0:
            Pb = goToBall(rb, vb, rg, rr, rl, self.vravg)

            if any(np.abs(rb) > rl):
                robot.field = UVF(Pb, direction=-np.sign(rb[1]), radius=0.07)
            else:
                robot.field = UVF(Pb, radius=0.10)
        else:
            robot.field = DirectionalField(self.attackAngle, nullgamma=True)