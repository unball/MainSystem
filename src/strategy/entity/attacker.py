from ..entity import Entity
from strategy.field.UVF import UVF
from strategy.field.DirectionalField import DirectionalField
from strategy.movements import goToBall
from tools import angError, howFrontBall, howPerpBall, ang, norml
from tools.interval import Interval
import numpy as np

class Attacker(Entity):
    def __init__(self, world, robot):
        super().__init__(world, robot)
        print("oi")
        self.lastDirectionChange = 0
        self.attackAngle = None
        self.attackState = 0
        self.vravg = 0

    def directionDecider(self):
        if self.robot.field is not None:
            ref_th = self.robot.field.F(self.robot.pose)
            rob_th = self.robot.th

            if abs(angError(ref_th, rob_th)) > 120 * np.pi / 180:
                self.robot.direction *= -1

    def fieldDecider(self):
        rr = np.array(self.robot.pos)
        vr = np.array(self.robot.v)
        self.vravg = 0.995 * self.vravg + 0.005 * norml(vr)
        rb = np.array(self.world.ball.pos)
        vb = np.array(self.world.ball.v)
        rg = np.array(self.world.field.goalPos)
        rl = np.array(self.world.field.marginPos)

        # Define estado do movimento
        if self.attackState == 0:
            if -howFrontBall(rb, rr, rg)  > 0 and abs(howPerpBall(rb, rr, rg)) < 0.075 * 0.75 and abs(angError(self.robot.th, ang(rb, rg))) < 15 * np.pi / 180:
                self.attackState = 1
                self.attackAngle = ang(rb, rg)
            else: self.attackState = 0
        else:
            if -howFrontBall(rb, rr, rg)  > 0 and abs(howPerpBall(rb, rr, rg)) < 0.075*2 and abs(angError(self.robot.th, ang(rb, rg))) < 90 * np.pi / 180:
                self.attackState = 1
            else:
                self.attackState = 0

        # Aplica o movimento
        if self.attackState == 0:
            Pb = goToBall(rb, vb, rg, rr, rl, self.vravg)

            if any(np.abs(rb) > rl):
                self.robot.field = UVF(Pb, direction=-np.sign(rb[1]), radius=0.07)
            else:
                self.robot.field = UVF(Pb, radius=0.10)
        else:
            self.robot.field = DirectionalField(self.attackAngle)