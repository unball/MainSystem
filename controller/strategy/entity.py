from abc import ABC, abstractmethod
from controller.strategy.movements import goToBall, goToGoal, projectBall, howFrontBall, howPerpBall, goalkeep, blockBallElipse
from controller.strategy.field import UVFDefault, GoalKeeperField, DefenderField, UVFavoidGoalArea
from controller.tools import ang, angError, norm, unit
import numpy as np

class Entity(ABC):
    def __init__(self, robot, color):
        super().__init__()

        self.robot = robot
        """Robô associado a esta entidade"""

        self.color = color

    def directionDecider(self):
        """Altera a propriedade `dir` do robo de acordo com a decisão"""
        # Inverte se o último erro angular foi maior que 160º
        if abs(self.robot.lastAngError) > 160 * np.pi / 180:
            self.robot.dir *= -1

    @abstractmethod
    def movementDecider(self):
        """Altera a propriedade `field` do robo de acordo com a decisão"""
        pass

    @property
    def name(self):
        return self.__class__.__name__

class Attacker(Entity):
    def __init__(self, world, robot):
        super().__init__(robot, (0,0,255))

        self.world = world
        self.movState = 0
        self.rg = (0,0)

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
        vr = np.array(self.robot.lastControlLinVel * unit(self.robot.th))

        # Bola projetada com offset
        rbpo = projectBall(rb, vb, rr, rg, self.world.marginLimits)

        # Ângulo da bola até o gol
        ballGoalAngle = ang(rb, rg)

        # Ângulo do robô até a bola
        robotBallAngle = ang(rr, rb)

        # Se estiver atrás da bola, estiver em uma faixa de distância "perpendicular" da bola, estiver com ângulo para o gol com erro menor que 30º vai para o gol
        if howFrontBall(rb, rr, rg) < -0.03*(1-self.movState) and abs(howPerpBall(rb, rr, rg)) < 0.045 + self.movState*0.05 and abs(angError(ballGoalAngle, rr[2])) < (30+self.movState*50)*np.pi/180:
            pose, gammavels = goToGoal(rg, rr, vr)
            self.robot.vref = 999
            self.robot.gammavels = gammavels
            self.movState = 1
        # Se não, vai para a bola
        else:
            pose, gammavels = goToBall(rb, rg, vb, self.world.marginLimits)
            self.robot.vref = 999
            self.robot.gammavels = gammavels
            self.movState = 0
        
        # Decide quais espirais estarão no campo e compõe o campo
        #if abs(rb[0]) > self.world.xmaxmargin: self.world.goalpos = (-self.world.goalpos[0], self.world.goalpos[1])

        # Muda o campo no gol caso a bola esteja lá
        if self.world.ball.insideGoalArea():
            self.robot.field = UVFavoidGoalArea(self.world, pose, rr)

        elif any(np.abs(rb) > self.world.marginLimits):
            self.robot.field = UVFDefault(self.world, (*pose[:2], 0), rr, direction=-np.sign(rb[1]), radius=0)
        else: 
            #if howFrontBall(rb, rr, rg) > 0: radius = 0
            #else: radius = None
            self.robot.field = UVFDefault(self.world, pose, rr, direction=0)

class Defender(Entity):
    def __init__(self, world, robot):
        super().__init__(robot, (0,255,0))

        self.world = world
    
    def directionDecider(self):
        """Altera a propriedade 'dir' do robô de acordo com a decisão"""
        # Inverte se o último erro angular foi maior que 90º
        if abs(self.robot.lastAngError) > 90 * np.pi / 180:
            self.robot.dir *= -1

    def movementDecider(self):
        # Dados necessários para a decisão
        rb = np.array(self.world.ball.pos.copy())
        vb = np.array(self.world.ball.vel.copy())
        rr = np.array(self.robot.pose)

        pose = blockBallElipse(rb, vb, rr)

        self.robot.vref = 0
        self.robot.field = UVFavoidGoalArea(self.world, pose, rr)
        #self.robot.field = UVFDefault(self.world, pose, rr, direction = 0)

        #self.robot.field = DefenderField(pose)

class GoalKeeper(Entity):
    def __init__(self, world, robot):
        super().__init__(robot, (255,0,0))

        self.world = world

    def directionDecider(self):
        """Altera a propriedade `dir` do robo de acordo com a decisão"""
        # Inverte se o último erro angular foi maior que 160º
        if abs(self.robot.lastAngError) > 90 * np.pi / 180:
            self.robot.dir *= -1

    def movementDecider(self):
        # Dados necessários para a decisão
        rb = np.array(self.world.ball.pos.copy())
        vb = np.array(self.world.ball.vel.copy())
        rr = np.array(self.robot.pose)
        rg = np.array(self.world.allyGoalPos)+[0.1,0]

        pose = goalkeep(rb, vb, rr, rg)
        
        self.robot.gammavels = (0,0,0)
        self.robot.vref = 0
        if np.abs(rr[0]-rg[0]) > 0.04:
            self.robot.field = GoalKeeperField(pose)
        else: self.robot.field = GoalKeeperField((rr[0], *pose[1:3]))
        #self.robot.field = UVFDefault(self.world, pose, direction=0, radius=0.14)