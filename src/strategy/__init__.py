from .entity.attacker import Attacker
from .entity.goalKeeper import GoalKeeper
from .entity.defender import Defender
from client.protobuf.vssref_common_pb2 import Foul
from client.referee import RefereeCommands
from tools import sats
import numpy as np
import time

class Strategy:
    def __init__(self, world):
        self.world = world
        self.formations = {
            "insane": (Attacker, Attacker, Attacker),
            "ambitious": (Attacker, Attacker, GoalKeeper),
            "safe": (Attacker, Defender, GoalKeeper)
        }

        # Variáveis de estado do formationDecider
        self.experimentStartTime = 0
        self.experimentPeriod = 5 # seconds
        self.ambitiousScore = 0.5
        self.safeScore = 0.5
        self.safeLineX = -0.20
        self.safeLineHyst = 0.1
        self.safeLineState = "learn"
        self.learnState = "safe"

        # Métricas do experimento
        self.initialBalance = 0
        self.ballAverageX = 0
        self.ballAverageX_n = 0
        
    def updateScores(self):
        ballAverageXScore = (self.ballAverageX) / (2 * self.world.field.maxX)
        balanceScore = sats((self.world.balance - self.initialBalance) / (2 * 2), 0, 1)

        deltaScore = (ballAverageXScore * 0.3 + balanceScore * 0.7) * 0.1

        if self.learnState == "safe":
            self.safeScore = max(min(self.safeScore + deltaScore, 1), 0)
        else:
            self.ambitiousScore = max(min(self.ambitiousScore + deltaScore, 1), 0)

        print("Scores: ")
        print("safe: " + str(self.safeScore))
        print("ambitious: " + str(self.ambitiousScore))

    def initExperiment(self):
        print("Iniciando experimento")
        # Atualiza os escores
        self.updateScores()
        
        # Decide o próximo estado com base na "adequação"
        if np.random.uniform() >= self.ambitiousScore / (self.ambitiousScore + self.safeScore):
            print("Modo ambitious")
            self.learnState = "ambitious"
        else:
            print("Modo safe")
            self.learnState = "safe"

        # Variáveis de início do experimento
        self.initialBalance = self.world.balance
        self.ballAverageX = 0
        self.ballAverageX_n = 0

        # Atualiza tempo de início
        self.experimentStartTime = time.time()
        
    def formationDecider(self):
        if np.abs(self.world.allyGoals - self.world.enemyGoals) >= 8: 
            self.experimentStartTime = 0
            return self.formations["insane"]
        else:
            rb = np.array(self.world.ball.pos)

            # Atualiza métricas
            self.ballAverageX = (self.ballAverageX * self.ballAverageX_n + rb[0]) / (self.ballAverageX_n + 1)
            self.ballAverageX_n += 1

            # Decide o estado
            if self.safeLineState == "learn":
                if rb[0] < self.safeLineX - self.safeLineHyst: self.safeLineState = "safe"
            elif self.safeLineState == "safe":
                if rb[0] > self.safeLineX + self.safeLineHyst: self.safeLineState = "learn"

            # Inicia um novo experimento
            if time.time() - self.experimentStartTime > self.experimentPeriod:
                self.initExperiment()

            # Decide a formação, finalmente...
            if self.safeLineState == "safe": return self.formations["safe"]
            else: return self.formations[self.learnState]
        
    def manageReferee(self, command):
        if command is None: return
        # Verifica gol
        if command.foul == Foul.KICKOFF:
            if RefereeCommands.color2side(command.teamcolor) == self.world.field.side:
                self.world.addEnemyGoal()
            elif RefereeCommands.color2side(command.teamcolor) == -self.world.field.side:
                self.world.addAllyGoal()
                

    def entityDecider(self, formation):
        self.world.team[0].updateEntity(formation[0])
        self.world.team[1].updateEntity(formation[1])
        self.world.team[2].updateEntity(formation[2])
    
    def update(self):
        formation = self.formationDecider()
        self.entityDecider(formation)
        for robot in self.world.team:
            if robot.entity is not None:
                robot.updateSpin()
                robot.entity.directionDecider()
                robot.entity.fieldDecider()

class EnemyStrategy:
    def __init__(self, world):
        self.world = world

    def entityDecider(self):
        self.world.team[0].updateEntity(Attacker)
        self.world.team[1].updateEntity(GoalKeeper)
        self.world.team[2].updateEntity(Defender)
    
    def update(self):
        self.entityDecider()
        for robot in self.world.team:
            if robot.entity is not None:
                robot.updateSpin()
                robot.entity.directionDecider()
                robot.entity.fieldDecider()