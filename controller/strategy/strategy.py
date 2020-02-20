from controller.strategy.entity import Attacker, GoalKeeper, Defender, MidFielder
from controller.tools import angl, angError, norm, unit
import numpy as np

class Strategy:
    def __init__(self, world, robots):
        self.robots = robots
        """Lista com os robôs a serem manipulados pela estratégia"""

        self.world = world
        self.state = 1

    def run(self):
        # Decisor de entidades
        self.entityDecider()

        # Decisor de direção
        self.directionDecider()

        # Decisor de movimento
        self.movementDecider()

    def goodPositionToAttack(self, robot1, robot2):
        rb = np.array(self.world.ball.pos.copy())
        r0b = rb - np.array(robot1.pos)
        r1b = rb - np.array(robot2.pos)
        d0b = np.linalg.norm(r0b)
        d1b = np.linalg.norm(r1b)
        a0b = angError(angl(r0b), robot1.th)
        a1b = angError(angl(r1b), robot2.th)
        # Robô 0 é um bom atacante
        if not robot2.isAlive() or (2*d0b < d1b and abs(a0b) < np.pi / 4 and abs(a0b) < abs(a1b)):
            return 0
        # Robô 1 é um bom atacante
        elif not robot1.isAlive() or (2*d1b < d0b and abs(a1b) < np.pi / 4 and abs(a1b) < abs(a0b)):
            return 1
        else:
        # Mantém o estado
            return -1

    def entityDecider(self):
        dynamicAttackerDefenderRobots = []
        for robot in self.robots:
            if robot.preferedEntity == "Atacante":
                robot.entity = Attacker(self.world, robot)
            elif robot.preferedEntity == "Zagueiro":
                robot.entity = Defender(self.world, robot)
            elif robot.preferedEntity == "Goleiro":
                robot.entity = GoalKeeper(self.world, robot)
            elif robot.preferedEntity == "AtacanteZagueiro":
                dynamicAttackerDefenderRobots.append(robot)
            elif robot.preferedEntity == "Meio Campo":
                robot.entity = MidFielder(self.world, robot)
                
        #self.robots[0].entity = Defender(self.world, self.robots[0])
        #self.robots[1].entity = Attacker(self.world, self.robots[1])
        #TODO: Caso o atacante esteja parado (sem a bola), Defender vira atacante
        # if self.world.ball.pos[0] < 0:
        #     self.robots[1].entity = Attacker(self.world, self.robots[1])
        if len(dynamicAttackerDefenderRobots) == 2:
            self.attackerDefenderDecider(*dynamicAttackerDefenderRobots)

    def attackerDefenderDecider(self, robot1, robot2):
        state = self.goodPositionToAttack(robot1, robot2)
        if state != -1:
            self.state = state
        if self.state == 0:
            robot1.entity = Attacker(self.world, robot1)
            robot2.entity = Defender(self.world, robot2)
        else:
            robot1.entity = Defender(self.world, robot1)
            robot2.entity = Attacker(self.world, robot2)

    def directionDecider(self):
        for robot in self.robots:
            if robot.entity is not None: robot.entity.directionDecider()

    def movementDecider(self):
        for robot in self.robots:
            if robot.entity is not None: robot.entity.movementDecider()