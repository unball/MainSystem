from .elements import *

class World:
    def __init__(self, n_robots=5):
        self.team = [TeamRobot(i) for i in range(n_robots)]
        self.enemies = [Robot(i) for i in range(n_robots)]
        self.ball = Ball()

    def update(self, message):
        teamPos = zip(message["ally_x"], message["ally_y"], message["ally_th"])
        enemiesPos = zip(message["enemy_x"], message["enemy_y"], message["enemy_th"])

        for robot, pos in zip(self.team, teamPos): robot.update(*pos)
        for robot, pos in zip(self.enemies, enemiesPos): robot.update(*pos)
        self.ball.update(message["ball_x"], message["ball_y"])
