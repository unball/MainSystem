from abc import ABC, abstractmethod
from controller.tools import ang, unit, angl, angError
import numpy as np

class Obstacle(ABC):
  def __init__(self):
    super().__init__()
  
  @abstractmethod
  def d(self, P: tuple) -> float:
    """Calcula a distância de um ponto `P` ao obstáculo"""
    pass
  
  @abstractmethod
  def F(self, P: tuple) -> float:
    """Calcula o campo gerado no ponto `P` pelo obstáculo"""
    pass

class HorizontalObstacle(Obstacle):
  def __init__(self, world, Pb: tuple):
    super().__init__()

    self.world = world
    self.Pb = Pb

  def d(self, P: tuple) -> float:
    return np.exp(-(self.world.ymax-abs(P[1]))**2/0.15**2)
    
  def F(self, P: tuple) -> float:
    return (np.arctan((self.Pb[0]-P[0])/0.15)-np.pi/2) * np.sign(P[1])

class LeftObstacle(Obstacle):
  def __init__(self, world, Pb: tuple):
    super().__init__()

    self.world = world
    self.Pb = Pb

  def d(self, P: tuple) -> float:
    return np.exp(-(self.world.xmax+P[0])**2/0.15**2)
    
  def F(self, P: tuple) -> float:
    return np.arctan((self.Pb[1]-P[1])/0.15)

class RightObstacle(Obstacle):
  def __init__(self, world, Pb: tuple, avoidGoal=True):
    super().__init__()

    self.world = world
    self.Pb = Pb
    self.avoidGoal = avoidGoal

  def d(self, P: tuple) -> float:
    return np.exp(-(self.world.xmax-P[0])**2/0.15**2) * (1-np.exp(-(P[1])**2/0.2**2)) if not self.avoidGoal else np.exp(-(self.world.xmax-P[0])**2/0.15**2)
    
  def F(self, P: tuple) -> float:
    return np.arctan((-P[1])/0.02) if not self.avoidGoal else (-np.arctan((self.Pb[1]-P[1])/0.15)-np.pi)

class PointObstacle(Obstacle):
  def __init__(self, world, Pb: tuple, Po: tuple):
    super().__init__()

    self.world = world
    self.Pb = Pb
    self.Po = Po

  def d(self, P: tuple) -> float:
    return np.exp(-((P[0]-self.Po[0])**2+(P[1]-self.Po[1])**2)/0.15**2)
    
  def F(self, P: tuple) -> float:
    if angError(ang(P, self.Po), ang(self.Po, self.Pb)) >= 0:
      return self.CCW(P[0]-self.Po[0], P[1]-self.Po[1])
    else:
      return self.CW(P[0]-self.Po[0], P[1]-self.Po[1])

  def CCW(self, x: float, y: float) -> float:
    return np.arctan2((x+y*(0.2**2-x**2-y**2)), (-y+x*(0.2**2-x**2-y**2)))

  def CW(self, x: float, y: float) -> float:
    return np.arctan2((-x+y*(0.2**2-x**2-y**2)), (y+x*(0.2**2-x**2-y**2)))


class UVF:
  """Classe que implementa um campo UVF"""
  def __init__(self, Pb: tuple, world, h: float=0.5, n: float=1, avoidGoal=True, pointObstacles=[]):

    self.h = h
    """Distância do ponto guia"""
    
    self.n = n
    """Parâmetro `n`"""
    
    self.world = world
    """Referência ao mundo"""
    
    self.Pb = Pb
    """Ponto final"""

    self.obstacles = [
      HorizontalObstacle(world, Pb),
      LeftObstacle(world, Pb),
      RightObstacle(world, Pb, avoidGoal)
    ]

    for pointObstacle in pointObstacles: self.obstacles.append(PointObstacle(world, Pb, pointObstacle))

  def F(self, P: tuple, Pb=None):
    """Calcula o campo no ponto `P`. Recebendo `Pb`, a posição do target final"""
    
    if Pb is None: Pb = self.Pb
    
    # Calcula o campo UVF de chegada
    targetAngle = ang(P, Pb) - self.n * (ang(P, self.Pg(Pb)) - ang(P, Pb))
    
    # Calcula o campo UVF de desvio de obstáculos
    avoidanceVector = np.array([0,0],dtype=np.float)
    avoidanceWeights = []
    for obstacle in self.obstacles:
      fo = obstacle.d(P)
      avoidanceVector += fo * unit(obstacle.F(P))
      avoidanceWeights.append(fo)

    avoidanceAngle = angl(avoidanceVector)
    
    # Une os campos
    f = max(avoidanceWeights)
    angle = angl(unit(targetAngle) * (1-f) + f * unit(avoidanceAngle))

    return angle
    
  def Pg(self, Pb: tuple):
    """Retorna o ponto guia"""
    return Pb[:2] + self.h * unit(Pb[2])

  def phi(self, P: tuple, d=0.0001):
    """Calcula o ângulo \\(\\phi = \\frac{\\partial \\theta_d}{\\partial x} \\cos(\\theta) + \\frac{\\partial \\theta_d}{\\partial y} \\sin(\\theta)\\) usado para o controle"""
    P = np.array(P)
    dx = (self.F(P+[d,0,0])-self.F(P))/d
    dy = (self.F(P+[0,d,0])-self.F(P))/d
    return dx*np.cos(P[2]) + dy*np.sin(P[2])

  def gamma(self, P: tuple, d=0.0001):
    P = np.array(P)
    Pb = np.array(self.Pb)
    return (self.F(P, Pb=Pb+[d,0,0])-self.F(P))/d * self.world.ball.inst_vx +\
           (self.F(P, Pb=Pb+[0,d,0])-self.F(P))/d * self.world.ball.inst_vy
