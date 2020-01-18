from controller.world.robot import Robot
from controller.world.ball import Ball
import time
import numpy as np

class Field():
  LEFT = -1
  """Lado aliado está a esquerda"""
  
  RIGHT = 1
  """Lado aliado está a direita"""

class World():
  """Classe de mundo que armazena as posições dos robôs, velocidades, posição da bola, limites de campo e escore de jogo."""
  
  def __init__(self):
    self.field_x_length = 1.66
    self.field_y_length = 1.30
    self.xmax = (self.field_x_length) / 2
    self.ymax = (self.field_y_length) / 2
    self.xmaxmargin = self.xmax - 0.15
    self.ymaxmargin = self.ymax - 0.15
    self.goalylength = 0.4
    self.n_robots = 5
    self.fieldSide = Field.RIGHT
    self.running = False
    self.robots = [Robot() for i in range(2*self.n_robots)]
    self.ball = Ball()
    self.__referenceTime = 0
    
  def update(self, visionMessage):
    """Recebe uma mensagem da visão e atualiza as posições e velocidades de robôs e bola"""
  
    # Atualiza cada robô localizado e identificado
    for i in range(visionMessage.nRobots):
      if not visionMessage.found[i]: continue
      
      # O ângulo do robô não pode variar de um loop para outro mais que 70% de pi/2, se isso ocorrer deve ser algum erro da visão
      #if self.robots[i].poseDefined and np.arccos(np.cos(visionMessage.th[i]-self.robots[i].th)) > 0.5*np.pi/2:
      #  theta = self.robots[i].th
      #else:
      theta = visionMessage.th[i]
      
      self.robots[i].update(visionMessage.x[i], visionMessage.y[i], theta)
    
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
