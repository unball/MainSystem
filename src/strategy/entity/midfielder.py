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

class Midfielder(Entity):
    def __init__(self, world, robot, 
                 perpBallLimiarTrackState = 0.075 * 0.5, 
                 perpBallLimiarAtackState = 0.075 * 2, 
                 alignmentAngleTrackState = 30, 
                 alignmentAngleAtackState = 90, 
                 spiralRadius = 0.04, 
                 spiralRadiusCorners = 0.07, 
                 approximationSpeed = 0, 
                 ballOffset = -0.03,
                 midfielderOffset = 0.45
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
        self.midfielderOffset = midfielderOffset

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
        return
        if self.robot.field is not None:
            ref_th = self.robot.field.F(self.robot.pose)
            rob_th = self.robot.th

            if abs(angError(ref_th, rob_th)) > 120 * np.pi / 180:# and time.time()-self.lastChat > .3:
                self.robot.direction *= -1
                self.lastChat = time.time()
            
            # Inverter a direção se o robô ficar preso em algo
            elif not self.robot.isAlive() and self.robot.spin == 0:
                if time.time()-self.lastChat > .3:
                    self.lastChat = time.time()
                    self.robot.direction *= -1
    
    def conditionAlignment(self, rb, rr, rg):
        return -howFrontBall(rb, rr, rg)  > 0 and abs(howPerpBall(rb, rr, rg)) < self.perpBallLimiarTrackState and abs(angError(self.robot.th, ang(rb, rg))) < self.alignmentAngleTrackState * np.pi / 180

    def alignedToGoal(self, rb, rr, rg):
        rg_up = rb + [0, 0.12]
        rg_down = rb + [0, -0.12]
        rg_up_plus = rb + [0, 0.18]
        rg_down_plus = rb + [0, -0.18]
        return self.conditionAlignment(rb, rr, rg) or self.conditionAlignment(rb, rr, rg_down) or self.conditionAlignment(rb, rr, rg_up) or self.conditionAlignment(rb, rr, rg_down_plus) or self.conditionAlignment(rb, rr, rg_up_plus)


    def angleToAttack(self, rr, rb, rg):
        rg_up = rb + [0, 0.12]
        rg_down = rb + [0, -0.12]
        rg_up_plus = rb + [0, 0.18]
        rg_down_plus = rb + [0, -0.18]
        if (-howFrontBall(rb, rr, rg_up)  > 0 and abs(howPerpBall(rb, rr, rg_up)) < self.perpBallLimiarTrackState and abs(angError(self.robot.th, ang(rb, rg_up))) < self.alignmentAngleTrackState * np.pi / 180):
            return ang(rr, rg_up)
        elif (-howFrontBall(rb, rr, rg_down)  > 0 and abs(howPerpBall(rb, rr, rg_down)) < self.perpBallLimiarTrackState and abs(angError(self.robot.th, ang(rb, rg_down))) < self.alignmentAngleTrackState * np.pi / 180):
            return ang(rr, rg_down)
        elif (-howFrontBall(rb, rr, rg_down_plus)  > 0 and abs(howPerpBall(rb, rr, rg_down_plus)) < self.perpBallLimiarTrackState and abs(angError(self.robot.th, ang(rb, rg_down_plus))) < self.alignmentAngleTrackState * np.pi / 180):
            return ang(rr, rg_down_plus)
        elif (-howFrontBall(rb, rr, rg_up_plus)  > 0 and abs(howPerpBall(rb, rr, rg_up_plus)) < self.perpBallLimiarTrackState and abs(angError(self.robot.th, ang(rb, rg_up_plus))) < self.alignmentAngleTrackState * np.pi / 180):
            return ang(rr, rg_up_plus)
        else:
            return ang(rr, rg)

    def conditionAlignmentRelaxed(self, rb, rr, rg):
        return -howFrontBall(rb, rr, rg)  > 0 and abs(howPerpBall(rb, rr, rg)) < self.perpBallLimiarAtackState and abs(angError(self.robot.th, ang(rb, rg))) < self.alignmentAngleAtackState * np.pi / 180

    def alignedToGoalRelaxed(self, rb, rr, rg):
        rg_up = rb + [0, 0.12]
        rg_down = rb + [0, -0.12]
        rg_up_plus = rb + [0, 0.18]
        rg_down_plus = rb + [0, -0.18]
        return self.conditionAlignmentRelaxed(rb, rr, rg) or self.conditionAlignmentRelaxed(rb, rr, rg_up) or self.conditionAlignmentRelaxed(rb, rr, rg_down) or self.conditionAlignmentRelaxed(rb, rr, rg_down_plus) or self.conditionAlignmentRelaxed(rb, rr, rg_up_plus)

    def alignedToBall(self, rb, rr):
        return (norm(rr, rb) < 0.10 or abs(angError(self.robot.th, ang(rr, rb))) < 30 * np.pi / 180) and np.abs(self.robot.th) < np.pi / 2 and np.abs(rb[1]) > 0.2

    def alignedToBallRelaxed(self, rb, rr):
        return (norm(rr, rb) < 0.15 or abs(angError(self.robot.th, ang(rr, rb))) < 70 * np.pi / 180) and np.abs(self.robot.th) < np.pi / 2

    def fieldDecider(self):
        # Variáveis úteis
        rr = np.array(self.robot.pos)
        vr = np.array(self.robot.v)
        rb = np.array(self.world.ball.pos)
        vb = np.array(self.world.ball.v)
        rg = np.array(self.world.field.goalPos)
        rl = np.array(self.world.field.size) - np.array([0, 0.14])

        # Atualiza histórico de velocidade do robô
        self.vravg = 0.995 * self.vravg + 0.005 * norml(vr)

        # Define estado do movimento
        # Ir até a bola
        if self.attackState == 0:
            if self.alignedToGoal(rb, rr, rg):
                self.attackState = 1
                self.attackAngle = self.angleToAttack(rr, rb, rg)
            elif self.alignedToBall(rb, rr):
                self.attackState = 2
                self.attackAngle = ang(rr, rb) # preciso melhorado
            else: self.attackState = 0

        # Ataque ao gol
        elif self.attackState == 1:
            if self.alignedToGoalRelaxed(rb, rr, rg):
                self.attackState =  1
            else:
                self.attackState = 0

        # Ataque à bola
        elif self.attackState == 2:
            if  self.alignedToBallRelaxed(rb, rr):
                self.attackState =  2
            else:
                self.attackState = 0

        # Movimento de alinhamento
        if self.attackState == 0:
            Pb = goToBall(rb, vb, rg, rr, rl, self.vravg, self.ballOffset)
            Pb = np.array([Pb[0]-self.midfielderOffset,Pb[1],Pb[2]])
            if np.abs(rb[1]) > rl[1]:
                self.robot.vref = math.inf
                self.robot.field = UVF(Pb, direction=-np.sign(rb[1]), radius=self.spiralRadiusCorners)
            else:
                self.robot.vref = self.approximationSpeed
                self.robot.field = UVF(Pb, radius=self.spiralRadius)
        
        # Movimento reto
        elif self.attackState == 1 or self.attackState == 2:
            self.robot.vref = math.inf
            self.robot.field = DirectionalField(self.attackAngle)

        # Campo para evitar área aliada
        a, b = self.world.field.areaEllipseSize
        center = self.world.field.areaEllipseCenter
        self.robot.field = AvoidanceField(self.robot.field, AvoidEllipse(center, 0.6*a, 0.80*b), borderSize=0.15)

        # Obtém outros aliados
        otherAllies = [robot for robot in self.world.team if robot != self.robot]
        enemies = [robot for robot in self.world.enemies]

        # Campo para evitar área inimiga
        if np.any([insideEllipse(robot.pos, a, b, rg) for robot in otherAllies]):
            self.robot.field = AvoidanceField(self.robot.field, AvoidEllipse(rg, 0.6*a, 0.80*b), borderSize=0.15)

        # Campo para evitar outro robô, (só se não estiver alinhado)
        if self.attackState == 0:
            for robot in otherAllies + enemies:
                self.robot.field = AvoidanceField(self.robot.field, AvoidCircle(robot.pos, 0.05), borderSize=0.10)

        # for robot in self.world.team:
        #     if robot.id != self.robot.id:
        #         if robot.entity.__class__.__name__ ==  "Attacker":
        #             if robot.entity.attackState != 0 and self.attackState == 0:
        #                 self.robot.field = AvoidanceField(self.robot.field, AvoidEllipse(rg, 0.6*a, 0.80*b), borderSize=0.15)
        #                 self.robot.field = AvoidanceField(self.robot.field, AvoidCircle(robot.pos, 0.05), borderSize=0.05)
        #             elif insideEllipse(robot.pos, a, b, rg):
        #             # elif norm(robot.pos, rb) < norm(rr, rb):
        #                 self.robot.field = AvoidanceField(self.robot.field, AvoidEllipse(rg, 0.6*a, 0.80*b), borderSize=0.15)
        #         else:
        #             self.robot.field = AvoidanceField(self.robot.field, AvoidCircle(robot.pos, 0.05), borderSize=0.05)


