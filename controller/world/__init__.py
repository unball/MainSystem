from model.paramsPattern import ParamsPattern
from controller.world.robot import Robot
from controller.world.ball import Ball
import time
import numpy as np

class Field():
  LEFT = -1
  """Lado aliado está a esquerda"""
  
  RIGHT = 1
  """Lado aliado está a direita"""

class World(ParamsPattern):
  """Classe de mundo que armazena as posições dos robôs, velocidades, posição da bola, limites de campo e escore de jogo."""
  
  def __init__(self):
    ParamsPattern.__init__(self, "worldConfig", {
      "UVF_r": 0.05,
      "UVF_Kr": 15,
      "UVF_Kr_single": 0.1,
      "UVF_horRepSize": 0.05,
      "UVF_horMinDist": 0.1,
      "UVF_verRepSize": 0.15,
      "UVF_verGoalSize": 0.2,
      "UVF_verMinDist": 0.15,
      "UVF_ponRadius": 0.3,
      "UVF_ponDistanceRadius": 0.1,
      "UVF_ponMinAvoidanceAngle": 0.5
    })

    self.field_x_length = 1.66
    self.field_y_length = 1.30
    self.xmax = (self.field_x_length) / 2
    self.ymax = (self.field_y_length) / 2
    self.xmaxmargin = self.xmax - 0.2
    self.ymaxmargin = self.ymax - 0.2
    self.marginLimits = (self.xmaxmargin, self.ymaxmargin)
    self.goalpos = (self.xmax, 0)
    self.goalylength = 0.4
    self.n_robots = 5
    self.fieldSide = Field.RIGHT
    self.running = False
    self.robots = [Robot() for i in range(self.n_robots)]
    self.enemyRobots = []
    self.edges = []
    self.ball = Ball()
    self.__referenceTime = 0
    
  def update(self, visionMessage):
    """Recebe uma mensagem da visão e atualiza as posições e velocidades de robôs e bola"""
  
    # Atualiza cada robô localizado e identificado
    for i,allyPose in enumerate(visionMessage.allyPoses):
      if not allyPose[3]: continue
      
      # O ângulo do robô não pode variar de um loop para outro mais que 70% de pi/2, se isso ocorrer deve ser algum erro da visão
      #if self.robots[i].poseDefined and np.arccos(np.cos(visionMessage.th[i]-self.robots[i].th)) > 0.5*np.pi/2:
      #  theta = self.robots[i].th
      #else:
      theta = allyPose[2]
      
      self.robots[i].update(allyPose[0], allyPose[1], theta)

    # Atualiza a lista de robôs adversários
    self.enemyRobots = visionMessage.advPos
    
    # Atualiza a bola se ela foi localizada
    if visionMessage.ball_found:
      self.ball.update(visionMessage.ball_x, visionMessage.ball_y)
    
    # Computa a velocidade com base no tempo passado desde a última chamada a `update` e atualiza o tempo para o tempo atual.
    dt = time.time() - self.__referenceTime
    self.calc_velocities(dt)
    self.__referenceTime = time.time()
    
  def setRunning(self, state):
    """Recebe uma flag `state` indicando se o jogo está ou não rodando."""
    self.running = state
    
  def getRobots(self):
    """Retorna os robôs."""
    return self.robots
    
  def calc_velocities(self, dt):
    """Computa as velocidades para robôs e bola com base em suas posições anteriores e atuais e com base no intervalo de tempo `dt`."""
    for robot in self.robots:
      robot.calc_velocities(dt)
    
    self.ball.calc_velocities(dt)

  def setEdges(self, points):
    self.edges = points