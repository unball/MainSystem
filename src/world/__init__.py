from .elements import *

class Field:
    def __init__(self, side):
        self.width = 1.75
        self.height = 1.35
        self.goalAreaWidth = 0.15
        self.goalAreaHeight = 0.70

        self.xmargin = 0.30
        self.ymargin = 0.18
        self.side = side

        self.areaEllipseSize = (0.35, 0.50)
        self.areaEllipseCenter = (-self.maxX + 0.10, 0)

    @property
    def maxX(self):
        return self.width / 2

    @property
    def maxY(self):
        return self.height / 2

    @property
    def size(self):
        return (self.maxX, self.maxY)

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

    @property
    def goalAreaSize(self):
        return (self.goalAreaWidth, self.goalAreaHeight)

class World:
    def __init__(self, n_robots=5, side=1, vss=None):
        self._team = [TeamRobot(self, i) for i in range(n_robots)]
        self.enemies = [TeamRobot(self, i) for i in range(n_robots)]
        self.ball = Ball(self)
        self.field = Field(side)
        self.vss = vss

        self.allyGoals = 0
        self.enemyGoals = 0

    def update(self, message):
        teamPos = zip(message["ally_x"], message["ally_y"], message["ally_th"], message["ally_vx"], message["ally_vy"], message["ally_w"])
        enemiesPos = zip(message["enemy_x"], message["enemy_y"], message["enemy_th"], message["enemy_vx"], message["enemy_vy"], message["enemy_w"])

        for robot, pos in zip(self.team, teamPos): robot.update(*pos)
        for robot, pos in zip(self.enemies, enemiesPos): robot.update(*pos)
        self.ball.update(message["ball_x"], message["ball_y"], message["ball_vx"], message["ball_vy"])

    def addAllyGoal(self):
        print("Gol aliado!")
        self.allyGoals += 1

    def addEnemyGoal(self):
        print("Gol inimigo!")
        self.enemyGoals += 1

    @property
    def goals(self):
        return self.allyGoals + self.enemyGoals

    @property
    def balance(self):
        return self.allyGoals - self.enemyGoals

    @property
    def team(self):
        return self._team#[robot for robot in self._team if robot.on]

    @property
    def raw_team(self):
        return self._team