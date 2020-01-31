from abc import ABC, abstractmethod
from controller.strategy.movements import goToBall, goToGoal, projectBall, howFrontBall
from controller.strategy.field import UVFDefault
from controller.tools import ang, angError, norm
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

    def directionDecider(self):
        # Por enquanto sempre vai para frente
        self.robot.dir = 1

    def movementDecider(self):
        # Dados necessários para a decisão
        rb = np.array(self.world.ball.pos.copy())
        vb = np.array(self.world.ball.vel.copy())
        ab = np.array(self.world.ball.acc.copy())
        rg = np.array(self.world.goalpos)
        rr = np.array(self.robot.pose)
        vr = max(self.robot.velmod, 0.08)

        # Bola projetada com offset
        rbpo = projectBall(rb, vb, ab, rr, vr, rg, self.world.xmax, self.world.ymax)

        # Ângulo da bola até o gol
        ballGoalAngle = ang(rb, rg)

        # Ângulo do robô até a bola
        robotBallAngle = ang(rr, rb)

        # Se estiver perto da bola, estiver atrás da bola e estiver com ângulo para o gol com erro menor que 50º vai para o gol
        if norm(rb, rr) < 0.18 and howFrontBall(rb, rr, rg) < -0.03 and abs(angError(ballGoalAngle, robotBallAngle)) < 30*np.pi/180  and abs(angError(ballGoalAngle, rr[2])) < 30*np.pi/180:
            pose = goToGoal(rg, rr)

        # Se não, vai para a bola
        else:
            pose = goToBall(rb, vb, ab, rr, vr, rg, self.world.xmax, self.world.ymax)
        # Cria-se o campo com base no pose
        self.robot.field = UVFDefault(self.world, pose)