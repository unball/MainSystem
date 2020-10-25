from ..entity import Entity
from strategy.field.UVF import UVF
from strategy.field.DirectionalField import DirectionalField
from strategy.field.areaAvoidance.avoidanceField import AvoidanceField
from strategy.field.areaAvoidance.area import AvoidCircle
from strategy.movements import goToBall
from tools import angError, howFrontBall, howPerpBall, ang, norml
from tools.interval import Interval
from control.UFC import UFC_Simple
import numpy as np
import math

class Attacker(Entity):
    def __init__(self, world, robot, 
                 perpBallLimiarTrackState = 0.075 * 0.75, 
                 perpBallLimiarAtackState = 0.075 * 2, 
                 alignmentAngleTrackState = 15, 
                 alignmentAngleAtackState = 90, 
                 spiralRadius = 0.10, 
                 spiralRadiusCorners = 0.07, 
                 approximationSpeed = 0.5, 
                 ballOffset = 0.015
        ):

        Entity.__init__(self, world, robot)
        
        # Params
        self.perpBallLimiarTrackState = perpBallLimiarTrackState
        self.perpBallLimiarAtackState = perpBallLimiarAtackState
        self.alignmentAngleTrackState = alignmentAngleTrackState
        self.alignmentAngleAtackState = alignmentAngleAtackState
        self.spiralRadius = spiralRadius
        self.spiralRadiusCorners = spiralRadiusCorners
        self.approximationSpeed = approximationSpeed
        self.ballOffset = ballOffset

        # States
        self.lastDirectionChange = 0
        self.attackAngle = None
        self.attackState = 0
        self.vravg = 0

        self._control = UFC_Simple(self.world)

    @property
    def control(self):
        return self._control

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
        #if self.side == -1: rg = (-rg[0], rg[1])
        rl = np.array(self.world.field.marginPos)

        # Define estado do movimento
        if self.attackState == 0:
            if -howFrontBall(rb, rr, rg)  > 0 and abs(howPerpBall(rb, rr, rg)) < self.perpBallLimiarTrackState and abs(angError(self.robot.th, ang(rb, rg))) < self.alignmentAngleTrackState * np.pi / 180:
                self.attackState = 1
                self.attackAngle = ang(rb, rg)
            else: self.attackState = 0
        else:
            if -howFrontBall(rb, rr, rg)  > 0 and abs(howPerpBall(rb, rr, rg)) < self.perpBallLimiarAtackState and abs(angError(self.robot.th, ang(rb, rg))) < self.alignmentAngleAtackState * np.pi / 180:
                self.attackState = 1
            else:
                self.attackState = 0

        # Aplica o movimento
        if self.attackState == 0:
            Pb = goToBall(rb, vb, rg, rr, rl, self.vravg, self.ballOffset)

            if any(np.abs(rb) > rl):
                self.robot.vref = math.inf
                self.robot.field = UVF((Pb[0], Pb[1]+np.sign(rb[1])*0.10, Pb[2]), direction=-np.sign(rb[1]), radius=self.spiralRadiusCorners)
            else:
                self.robot.vref = self.approximationSpeed
                self.robot.field = UVF(Pb, radius=self.spiralRadius)
        else:
            self.robot.vref = math.inf
            self.robot.field = DirectionalField(self.attackAngle)

        
        self.robot.field = AvoidanceField(self.robot.field, AvoidCircle((0,0), 0.10), borderSize=0.1)
    

