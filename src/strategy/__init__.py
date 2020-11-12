from abc import ABC
from .entity.attacker import Attacker
from .entity.goalKeeper import GoalKeeper
from .entity.defender import Defender
from .entity.midfielder import Midfielder
from client.protobuf.vssref_common_pb2 import Foul
from client.referee import RefereeCommands
from tools import sats, norml, unit, angl, angError
import numpy as np
import time

class Strategy(ABC):
    def __init__(self):
        super().__init__()

    def manageReferee(self, rp, command):
        if command is None: return

        # Verifica gol
        if command.foul == Foul.KICKOFF:
            if RefereeCommands.color2side(command.teamcolor) == self.world.field.side:
                self.world.addEnemyGoal()
            elif RefereeCommands.color2side(command.teamcolor) == -self.world.field.side:
                self.world.addAllyGoal()

        # Inicia jogo
        elif command.foul == Foul.GAME_ON:
            for robot in self.world.raw_team: robot.turnOn()
            
        # Pausa jogo
        elif command.foul == Foul.STOP:
            for robot in self.world.raw_team: robot.turnOff()

        #rp.send([])

class MainStrategy(Strategy):
    def __init__(self, world):
        super().__init__()

        self.world = world
        self.formations = {
            "insane": (Attacker, Attacker, Midfielder),
            "crazy": (Attacker, Midfielder, Defender), 
            "ambitious": (Attacker, Midfielder, GoalKeeper),
            "safe": (Attacker, Defender, GoalKeeper)
        }
        self.lastGoalkeeper = None
        self.goalkeeperIndx = None
        self.AttackerIdx = None

        # Variáveis de estado do formationDecider
        self.experimentStartTime = 0
        self.experimentPeriod = 5 # seconds
        self.ambitiousScore = 0.5
        self.safeScore = 0.5
        self.safeLineX = -0.20
        self.safeLineHyst = 0.1
        self.ambitiousLineX = 0.3
        self.ambitiousLineHyst = 0.1
        self.formationState = "learn"
        self.learnState = "safe"

        self.insaneBehavior = False

        # Métricas do experimento
        self.initialBalance = 0
        self.ballAverageX = 0
        self.ballAverageX_n = 0

    def manageReferee(self, rp, command):
        if command is None: return

        # Verifica gol
        if command.foul == Foul.KICKOFF:
            if RefereeCommands.color2side(command.teamcolor) == self.world.field.side:
                self.world.addEnemyGoal()
            elif RefereeCommands.color2side(command.teamcolor) == -self.world.field.side:
                self.world.addAllyGoal()

        elif command.foul == Foul.PENALTY_KICK:
            if RefereeCommands.color2side(command.teamcolor) != self.world.field.side:
                rg = -np.array(self.world.field.goalPos) * self.world.field.side
                rg[0] += 0.18 * self.world.field.side
                positions = [(0, (rg[0], rg[1], 90))]
                positions.append((1, (0,  0.30, 1.2*180)))
                positions.append((2, (0, -0.30, 0.8*180)))
                print(positions)
                rp.send(positions)
            else:
                rg = -np.array(self.world.field.goalPos) * self.world.field.side
                rg[0] += 0.18 * self.world.field.side
                positions = [(0, (rg[0], rg[1], 90))]
                penaltiPos = np.array([0.375, 0])
                ang = 15 
                robotPos = penaltiPos  - 0.065 * unit(ang*np.pi/180)
                positions.append((1, (robotPos[0],  robotPos[1], ang)))
                positions.append((2, (0, -0.30, 3)))
                print(positions)
                rp.send(positions)


        # Inicia jogo
        elif command.foul == Foul.GAME_ON:
            for robot in self.world.raw_team: robot.turnOn()
            
        # Pausa jogo
        elif command.foul == Foul.STOP:
            for robot in self.world.raw_team: robot.turnOff()
        
    def updateScores(self):
        ballAverageXScore = (self.ballAverageX) / (2 * self.world.field.maxX)
        balanceScore = sats((self.world.balance - self.initialBalance) / (2 * 2), 0, 1)

        deltaScore = (ballAverageXScore * 0.3 + balanceScore * 0.7) * 0.2

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

    def learnStateUpdate(self):
        rb = np.array(self.world.ball.pos)

        # Atualiza métricas
        self.ballAverageX = (self.ballAverageX * self.ballAverageX_n + rb[0]) / (self.ballAverageX_n + 1)
        self.ballAverageX_n += 1

        # Inicia um novo experimento
        if time.time() - self.experimentStartTime > self.experimentPeriod:
            self.initExperiment()

        
    def formationDecider(self):
        rb = np.array(self.world.ball.pos)

        # Decide estado do insane
        if self.insaneBehavior:
            if np.abs(self.world.balance) < 8:
                self.insaneBehavior = False
                # Condição inicial para a formação ao sair do modo insano
                if rb[0] < self.safeLineX: self.formationState = "safe"
                elif rb[0] > self.ambitiousLineX: self.formationState = "ambitious"
                else: self.formationState = "learn"
        else:
            if np.abs(self.world.balance) >= 8:
                self.insaneBehavior = True
                # Condição inicial para a formação ao entrar do modo insano
                if rb[0] < self.safeLineX: self.formationState = "ambitious"
                else: self.formationState = "insane"

        # Executa conforme estado
        if self.insaneBehavior:
            return self.formationDeciderInsane()
        else:
            return self.formationDeciderNormal()


    def formationDeciderNormal(self):
        rb = np.array(self.world.ball.pos)

        # Decide formação
        if self.formationState == "learn":
            if rb[0] < self.safeLineX - self.safeLineHyst: self.formationState = "safe"
            if rb[0] > self.ambitiousLineX + self.ambitiousLineHyst: self.formationState = "ambitious"
        elif self.formationState == "safe":
            if rb[0] > self.safeLineX + self.safeLineHyst: self.formationState = "learn"
        elif self.formationState == "ambitious":
            if rb[0] < self.ambitiousLineX - self.ambitiousLineHyst: self.formationState = "learn"
        else:
            print("ESTADO NORMAL INVÁLIDO")

        # Executa formação conforme o estado
        if self.formationState != "learn":
            return self.formations[self.formationState]
        else:
            self.learnStateUpdate()
            return self.formations[self.learnState]

    def formationDeciderInsane(self):
        rb = np.array(self.world.ball.pos)

        if self.formationState == "insane":
            if rb[0] < self.safeLineX - self.safeLineHyst: self.formationState = "ambitious"
        elif self.formationState == "ambitious":
            if rb[0] > self.safeLineX + self.safeLineHyst: self.formationState = "insane"
        else:
            print("ESTADO INSANO INVÁLIDO!")

        # Executa o estado
        return self.formations[self.formationState]

    def entityDecider(self, formation):
        if len(self.world.team) == 0: return
        form = list(formation)
        robots= self.world.team.copy()
        
        if GoalKeeper in formation:
            rg = - np.array(self.world.field.goalPos)
            dist = [norml(np.array(rr.pos)-rg) for rr in self.world.team]
            nearst = np.argmin(dist)
            minDist = np.min(dist)

            if(self.lastGoalkeeper != nearst):
                if(self.lastGoalkeeper is None or 2*minDist <= dist[self.lastGoalkeeper]):
                    self.goalkeeperIndx = nearst
                    robots[self.goalkeeperIndx].updateEntity(GoalKeeper)
            form.remove(GoalKeeper)
            _ = robots.pop(self.goalkeeperIndx)
        else:
            self.goalkeeperIndx = None
        
        if Attacker in formation:
            if self.goalkeeperIndx != None and \
               self.AttackerIdx != None and self.AttackerIdx > self.goalkeeperIndx:
                attackerIdx = self.AttackerIdx -1
            else:
                attackerIdx = self.AttackerIdx
            rb = np.array(self.world.ball.pos)
            dist = [norml(rb - np.array(rr.pos)) for rr in robots]
            nearst = np.argmin(dist)
            minDist = np.min(dist)
            if(attackerIdx != nearst):
                if(attackerIdx is None or 2*minDist <= dist[attackerIdx]):
                    angNearst = angError(angl(rb - np.array(robots[nearst].pos)), robots[nearst].th)
                    angLastAttacker = angError(angl(rb - np.array( robots[attackerIdx].pos)),  robots[attackerIdx].th) if not attackerIdx is None else 2*np.pi
                    if attackerIdx is None or (np.abs(angNearst) <= np.pi/4 and np.abs(angNearst) < np.abs(angLastAttacker)):
                        attackerIdx = nearst 
                        self.AttackerIdx = robots[nearst].id
                        robots[nearst].updateEntity(Attacker)
            form.remove(Attacker)
            robots[attackerIdx].updateEntity(Attacker)            
            _ = robots.pop(attackerIdx)
        else:
            self.AttackerIdx = None
        # quem sobra
        print(form)
        for r, e in zip(robots, form):
            r.updateEntity(e)

        print([robot.entity.__class__.__name__ for robot in self.world.team])

    
    def update(self):
        formation = self.formationDecider()
        self.entityDecider(formation)
        for robot in self.world.team:
            if robot.entity is not None:
                robot.updateSpin()
                robot.entity.fieldDecider()
                robot.entity.directionDecider()

class EnemyStrategy(Strategy):
    def __init__(self, world):
        super().__init__()
        self.world = world

    def entityDecider(self):
        formation = [Attacker, GoalKeeper, Defender]
        for i,robot in enumerate(self.world.team):
            robot.updateEntity(formation[i])
    
    def update(self):
        self.entityDecider()
        for robot in self.world.team:
            if robot.entity is not None:
                robot.updateSpin()
                robot.entity.directionDecider()
                robot.entity.fieldDecider()