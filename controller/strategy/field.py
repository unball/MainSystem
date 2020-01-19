from abc import ABC, abstractmethod
from controller.tools import ang, unit, angl
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

class UVF:
  """Classe que implementa um campo UVF"""
  def __init__(self, Pb: tuple, world, h: float=0.5, n: float=1, avoidGoal=True):

    self.h = h
    """Distância do ponto guia"""
    
    self.n = n
    """Parâmetro `n`"""
    
    self.world = world
    """Referência ao mundo"""
    
    self.Pb = Pb
    """Ponto final"""

    self.avoidGoal = avoidGoal

  def F(self, P: tuple, Pb=None):
    """Calcula o campo no ponto `P`. Recebendo `Pb`, a posição do target final"""
    
    if Pb is None: Pb = self.Pb
    
    # Calcula o campo UVF de chegada
    targetAngle = ang(P, Pb) - self.n * (ang(P, self.Pg(Pb)) - ang(P, Pb))
    
    # Calcula o campo UVF de desvio de obstáculos
    x = P[0]
    y = P[1]
    f1 = np.exp(-(self.world.ymax-abs(y))**2/0.15**2)
    f2 = np.exp(-(self.world.xmax+x)**2/0.15**2)
    f3 = np.exp(-(self.world.xmax-x)**2/0.15**2) * (1-np.exp(-(y)**2/0.2**2)) if not self.avoidGoal else np.exp(-(self.world.xmax-x)**2/0.15**2)
    avoidanceAngle = angl(
                     f1 * unit((np.arctan((Pb[0]-x)/0.15)-np.pi/2) * np.sign(y)) + \
                     f2 * unit((np.arctan((Pb[1]-y)/0.15))) + \
                     f3 * (unit(((np.arctan((-y)/0.02)))) if not self.avoidGoal else unit((-np.arctan((Pb[1]-y)/0.15)-np.pi)))
                     )
    
    # Une os campos
    f = max([f1,f2,f3])
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
