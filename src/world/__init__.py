from .elements import *

class Field:
    def __init__(self):
        self.width = 1.75
        self.height = 1.35
        self.xmargin = 0.30
        self.ymargin = 0.10

    @property
    def maxX(self):
        return self.width / 2

    @property
    def maxY(self):
        return self.height / 2

    @property
    def marginX(self):
        return self.maxX - self.xmargin
    
    @property
    def marginY(self):
        return self.maxY - self.ymargin

    @property
    def marginPos(self):
        return (self.marginX, self.marginY)

    @property
    def goalPos(self):
        return (self.maxX, 0)

class World:
    def __init__(self, n_robots=5):
        self.team = [TeamRobot(self, i) for i in range(n_robots)]
        self.enemies = [Robot(self, i) for i in range(n_robots)]
        self.ball = Ball(self)
        self.field = Field()

    def update(self, message):
        teamPos = zip(message["ally_x"], message["ally_y"], message["ally_th"])
        enemiesPos = zip(message["enemy_x"], message["enemy_y"], message["enemy_th"])

        for robot, pos in zip(self.team, teamPos): robot.update(*pos)
        for robot, pos in zip(self.enemies, enemiesPos): robot.update(*pos)
        self.ball.update(message["ball_x"], message["ball_y"])
