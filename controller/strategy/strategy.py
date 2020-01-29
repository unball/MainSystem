from controller.strategy.entity import Attacker

class Strategy:
    def __init__(self, world, robots):
        self.robots = robots
        """Lista com os robôs a serem manipulados pela estratégia"""

        self.world = world

    def run(self):
        # Decisor de entidades
        self.entityDecider()

        # Decisor de direção
        #self.directionDecider()

        # Decisor de movimento
        self.movementDecider()

    def entityDecider(self):
        # Todos são atacantes
        for robot in self.robots:
            robot.entity = Attacker(self.world, robot)

    def directionDecider(self):
        for robot in self.robots:
            if robot.entity is not None: robot.entity.directionDecider()

    def movementDecider(self):
        for robot in self.robots:
            if robot.entity is not None: robot.entity.movementDecider()