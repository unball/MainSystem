from abc import ABC, abstractmethod
from controller.strategy.movements import goToBall, goToGoal, projectBall, howFrontBall, howPerpBall
from controller.strategy.field import UVFDefault
from controller.tools import ang, angError, norm, unit
import numpy as np

class Entity(ABC):
    def __init__(self, robot):
        super().__init__()

        self.robot = robot
        """Robô associado a esta entidade"""

    @abstractmethod
    def directionDecider(self):
        """Altera a propriedade `dir` do robo de acordo com a decisão"""
        pass

    @abstractmethod
    def movementDecider(self):
        """Altera a propriedade `field` do robo de acordo com a decisão"""
        pass

class Attacker(Entity):
    def __init__(self, world, robot):
        super().__init__(robot)

        self.world = world
        self.movState = 0
        self.rg = (0,0)

    def directionDecider(self):
        # Inverte se o último erro angular foi maior que 160º
        # if abs(self.robot.lastAngError) > 160 * np.pi / 180:
        #     self.robot.dir *= -1
        self.robot.dir = 1

    def movementDecider(self):
        # Dados necessários para a decisão
        rb = np.array(self.world.ball.pos.copy())
        vb = np.array(self.world.ball.vel.copy())
        ab = np.array(self.world.ball.acc.copy())
        rr = np.array(self.robot.pose)
        # if self.movState == 0:
        #     self.rg = np.array(self.world.goalpos) + [0,0.15 / (np.pi/2) * np.arctan(rb[1] / 0.1)]
        # else:
        #     rg = self.rg
        # rg = self.rg
        rg = np.array(self.world.goalpos)
        vr = np.array(self.robot.velmod * unit(self.robot.th))

        # Bola projetada com offset
        rbpo = projectBall(rb, vb, rr, rg, self.world.marginLimits)

        # Ângulo da bola até o gol
        ballGoalAngle = ang(rb, rg)

        # Ângulo do robô até a bola
        robotBallAngle = ang(rr, rb)

        # Se estiver atrás da bola, estiver em uma faixa de distância "perpendicular" da bola, estiver com ângulo para o gol com erro menor que 30º vai para o gol
        if howFrontBall(rb, rr, rg) < -0.03*(1-self.movState) and abs(howPerpBall(rb, rr, rg)) < 0.045 + self.movState*0.03 and abs(angError(ballGoalAngle, rr[2])) < (30+self.movState*50)*np.pi/180:
            pose = goToGoal(rb, rg, rr)
            self.robot.vref = 999
            self.robot.gammavels = (0,0)
            self.movState = 1
        # Se não, vai para a bola
        else:
            pose = goToBall(rbpo, rg)
            self.robot.vref = 0
            self.robot.gammavels = (self.world.ball.inst_vx, self.world.ball.inst_vy)
            self.movState = 0
        
        # Decide quais espirais estarão no campo
        if abs(rb[1]) > self.world.ymaxmargin:
            direction = -np.sign(rb[1])
        else: direction = 0
        
        # Cria-se o campo com base no pose
        self.robot.field = UVFDefault(self.world, pose, direction)