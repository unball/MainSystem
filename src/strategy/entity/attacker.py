from ..entity import Entity
from strategy.field.UVF import UVF
from strategy.field.DirectionalField import DirectionalField
from strategy.field.areaAvoidance.avoidanceField import AvoidanceField
from strategy.field.areaAvoidance.avoidCircle import AvoidCircle
from strategy.field.areaAvoidance.avoidRect import AvoidRect
from strategy.field.areaAvoidance.avoidEllipse import AvoidEllipse
from strategy.movements import goToBall
from tools import angError, howFrontBall, howPerpBall, ang, norml, norm, insideEllipse
from tools.interval import Interval
from control.UFC import UFC_Simple
import numpy as np
import math
import time

class Attacker(Entity):
    def __init__(self, world, robot, 
                 perpBallLimiarTrackState = 0.075 * 0.5, 
                 perpBallLimiarAtackState = 0.075 * 2, 
                 alignmentAngleTrackState = 30, 
                 alignmentAngleAtackState = 90, 
                 spiralRadius = 0.07, 
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
        
        
        self.lastChat = 0

        self._control = UFC_Simple(self.world)
    @property
    def control(self):
        return self._control

    def directionDecider(self):
        if self.robot.field is not None:
            ref_th = self.robot.field.F(self.robot.pose)
            rob_th = self.robot.th

            if abs(angError(ref_th, rob_th)) > 120 * np.pi / 180 and time.time()-self.lastChat > .3:
                self.robot.direction *= -1
                self.lastChat = time.time()
            
            # Inverter a direção se o robô ficar preso em algo
            elif not self.robot.isAlive() and self.robot.spin == 0:
                if time.time()-self.lastChat > .3:
                    self.lastChat = time.time()
                    self.robot.direction *= -1
    
    def alignedToGoal(self, rb, rr, rg):
        return -howFrontBall(rb, rr, rg)  > 0 and abs(howPerpBall(rb, rr, rg)) < self.perpBallLimiarTrackState and abs(angError(self.robot.th, ang(rb, rg))) < self.alignmentAngleTrackState * np.pi / 180

    def alignedToGoalRelaxed(self, rb, rr, rg):
        return -howFrontBall(rb, rr, rg)  > 0 and abs(howPerpBall(rb, rr, rg)) < self.perpBallLimiarAtackState and abs(angError(self.robot.th, ang(rb, rg))) < self.alignmentAngleAtackState * np.pi / 180

    def alignedToBall(self, rb, rr):
        return (norm(rr, rb) < 0.10 or abs(angError(self.robot.th, ang(rr, rb))) < 30 * np.pi / 180) and np.abs(self.robot.th) < np.pi / 2 and np.abs(rb[1]) > 0.2

    def alignedToBallRelaxed(self, rb, rr):
        return (norm(rr, rb) < 0.15 or abs(angError(self.robot.th, ang(rr, rb))) < 70 * np.pi / 180) and np.abs(self.robot.th) < np.pi / 2

    def fieldDecider(self):
        rr = np.array(self.robot.pos)
        vr = np.array(self.robot.v)
        self.vravg = 0.995 * self.vravg + 0.005 * norml(vr)
        rb = np.array(self.world.ball.pos)
        vb = np.array(self.world.ball.v)
        rg = np.array(self.world.field.goalPos)
        #if self.side == -1: rg = (-rg[0], rg[1])
        rl = np.array(self.world.field.marginPos)

        goalAreaWidth, goalAreaHeight = self.world.field.goalAreaSize
        rga = np.array([-rg[0] - goalAreaWidth + 0.1, rg[1] - goalAreaHeight / 2])
        rgb = np.array([-rg[0] + goalAreaWidth + 0.3, rg[1] + goalAreaHeight / 2])

        # Define estado do movimento
        if self.attackState == 0: # Ir até a bola
            if self.alignedToGoal(rb, rr, rg):
                self.attackState = 1
                self.attackAngle = ang(rb, rg)
            elif self.alignedToBall(rb, rr):
                self.attackState = 2
                self.attackAngle = ang(rr, rb) # preciso melhorado
            else: self.attackState = 0

        elif self.attackState == 1: # Ataque ao gol
            if self.alignedToGoalRelaxed(rb, rr, rg):
                self.attackState =  1
            else:
                self.attackState = 0
        elif self.attackState == 2: # Ataque à bola
            if  self.alignedToBallRelaxed(rb, rr):
                self.attackState =  2
            else:
                self.attackState = 0


        
        # Executa spin se estiver morto
        #if not self.robot.isAlive():
        #    self.robot.setSpin(-np.sign(rr[0]) if rr[1] > 0 else np.sign(rr[0]))
        #    return

        # Aplica o movimento
        if self.attackState == 0:
            Pb = goToBall(rb, vb, rg, rr, rl, self.vravg, self.ballOffset)

            if any(np.abs(rb) > rl):
                self.robot.vref = math.inf
                self.robot.field = UVF((Pb[0], Pb[1]+np.sign(rb[1])*0.1, Pb[2]), direction=-np.sign(rb[1]), radius=self.spiralRadiusCorners)
            else:
                self.robot.vref = self.approximationSpeed
                self.robot.field = UVF(Pb, radius=self.spiralRadius)
        elif self.attackState == 1 or self.attackState == 2:
            self.robot.vref = math.inf
            self.robot.field = DirectionalField(self.attackAngle)

        
        #self.robot.field = AvoidanceField(self.robot.field, AvoidCircle((0,0), 0.10), borderSize=0.1)
        #self.robot.field = AvoidanceField(self.robot.field, AvoidRect(rga, rgb), borderSize=0.1)
        a, b = self.world.field.areaEllipseSize
        self.robot.field = AvoidanceField(self.robot.field, AvoidEllipse(self.world.field.areaEllipseCenter, 0.6*a, 0.80*b), borderSize=0.15)

        for robot in self.world.team:
            if robot.id != self.robot.id:
                if robot.entity.__class__.__name__ ==  "Attacker":
                    if robot.entity.attackState != 0 and self.attackState == 0:
                        self.robot.field = AvoidanceField(self.robot.field, AvoidEllipse(rg, 0.6*a, 0.80*b), borderSize=0.15)
                        self.robot.field = AvoidanceField(self.robot.field, AvoidCircle(robot.pos, 0.05), borderSize=0.05)
                    elif insideEllipse(robot.pos, a, b, rg):
                    # elif norm(robot.pos, rb) < norm(rr, rb):
                        self.robot.field = AvoidanceField(self.robot.field, AvoidEllipse(rg, 0.6*a, 0.80*b), borderSize=0.15)
                else:
                    self.robot.field = AvoidanceField(self.robot.field, AvoidCircle(robot.pos, 0.05), borderSize=0.05)


