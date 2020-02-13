from controller.strategy.entity import Attacker, GoalKeeper, Defender

class Strategy:
    def __init__(self, world, robots):
        self.robots = robots
        """Lista com os robôs a serem manipulados pela estratégia"""

        self.world = world

    def run(self):
        # Decisor de entidades
        self.entityDecider()

        # Decisor de direção
        self.directionDecider()

        # Decisor de movimento
        self.movementDecider()

    def entityDecider(self):
        self.robots[0].entity = Defender(self.world, self.robots[0])
        self.robots[1].entity = Attacker(self.world, self.robots[1])
        #TODO: Caso o atacante esteja parado (sem a bola), Defender vira atacante
        # if self.world.ball.pos[0] < 0:
        #     self.robots[1].entity = Attacker(self.world, self.robots[1])


    def directionDecider(self):
        for robot in self.robots:
            if robot.entity is not None: robot.entity.directionDecider()

    def movementDecider(self):
        for robot in self.robots:
            if robot.entity is not None: robot.entity.movementDecider()