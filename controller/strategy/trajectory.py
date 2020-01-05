from abc import ABC, abstractmethod
from controller.tools import norm, ang, angl
from model.paramsPattern import ParamsPattern
import scipy.optimize
import numpy as np
import matplotlib.pyplot as mlp
import math
import time
import dubins

class Trajectory(ABC):
  def __init__(self):
    super().__init__()

    self.lastt = 0
  
  @abstractmethod
  def P(self, t: float) -> tuple:
    """Função de parametrização da trajetória. Deve ser: \\(\\vec{P}(t) = (\\vec{r}(t), \\theta(t))\\), em que \\(\\|\\vec{r}(t)-\\vec{r}_0\\|\\) é contínua para todo \\(\\vec{r}_0\\) e \\(t \\in [0,1]\\)"""
    pass
  
  def tofindmin(self, t: float, r0: tuple) -> float:
    """Calcula  \\(\\|\\vec{r}(t)-\\vec{r}_0\\|\\)"""
    return norm(self.P(t), r0)
  
  def tofindroot(self, t: float, r0: tuple, d: float) -> float:
    """Calcula  \\(\\|\\vec{r}(t)-\\vec{r}_0\\| - d\\)"""
    return norm(self.P(t), r0) - d
    
  def tmin(self, r0: tuple):
    return scipy.optimize.brute(self.tofindmin, [(0,1)], args=(r0,))[0]
  
  def target(self, r0: tuple, d: float) -> tuple:
    """Função que retorna o target que dista `d` do ponto `r0` pertence à trajetória no sentido de progressão do percurso. Faz isso encontrando \\(t_{opt}\\) tal que: \\(\\|\\vec{r}(t_{opt})-\\vec{r}_0\\| - d = 0\\) usando o método de Brent e \\(t \\in [t_{min}, 1]\\) e retorna \\(\\vec{P}(t_{opt})\\)"""
    tmin = max(self.tmin(r0), self.lastt)
    dmin = self.tofindmin(tmin, r0)
    
    try:
      t = scipy.optimize.brentq(self.tofindroot, tmin, 1, args=(r0,max(dmin,d)))
    except:
      t = 1
    
    self.lastt = t

    return self.P(t)
  
  def interestPoints(self):
    return []

class StraightLine(Trajectory):
  """Classe que implementa uma trajetória reta"""
  def __init__(self, ra: tuple, rb: tuple):
    super().__init__()
    
    self.__ra = ra
    """Posição inicial"""
    
    self.__rb = rb
    """Posição final"""
    
  def P(self, t: float) -> tuple:
    """Implementa uma linha reta que liga os pontos \\(\\vec{r_a}\\) e \\(\\vec{r_b}\\)"""
    if t < 0: return self.__ra
    elif t > 1: return self.__rb
    x = t*self.__rb[0] + (1-t)*self.__ra[0]
    y = t*self.__rb[1] + (1-t)*self.__ra[1]
    o = np.arctan2(self.__rb[1]-self.__ra[1], self.__rb[0]-self.__ra[0])
    return (x,y,o)

  
class Dubins(Trajectory):
  """Classe que implementa uma trajetória dubins"""
  def __init__(self, ra: tuple, rb: tuple, radius: float):
    super().__init__()
    
    self.__ra = ra
    """Pose inicial"""
    
    self.__rb = rb
    """Pose final"""
    
    self.__radius = radius
    """Raio mínimo dos círculos na trajetória dubins"""
    
    self.__path = dubins.shortest_path(ra, rb, radius)
    
  def P(self, t: float) -> tuple:
    """Implementa trajetória dubins que liga os pontos \\(\\vec{r_a}\\) e \\(\\vec{r_b}\\) com raio `__radius`"""
    if t < 0: return self.__ra
    elif t > 1: return self.__rb
    return self.__path.sample(t*self.__path.path_length())

  def referenceAngle(self, robotPose: tuple, step: float):
    """Computa um ângulo de referência \\(\\theta_d\\) dado um pose \\((x,y,\\theta)\\) e um step, este ângulo é o ângulo entre o `robotPose` e o ponto `self.target(robotPose, step)`"""
    target = self.target(robotPose, step)
    return ang(robotPose, target)

  def phi(self, pose: tuple, step: float, d=0.0001):
    """Calcula o ângulo \\(\\phi = \\frac{\\partial \\theta_d}{\\partial x} \\cos(\\theta) + \\frac{\\partial \\theta_d}{\\partial y} \\sin(\\theta)\\) usado para o controle"""
    pose = np.array(pose)
    dx = (self.referenceAngle(pose+[d,0,0], step)-self.referenceAngle(pose, step)) / d
    dy = (self.referenceAngle(pose+[0,d,0], step)-self.referenceAngle(pose, step)) / d
    return dx*np.cos(pose[2]) + dy*np.sin(pose[2])


class Point(Trajectory):
  """Classe que implementa uma trajetória que contém apenas um ponto"""
  def __init__(self, p: tuple):
    super().__init__()

    self.__p = p
    """Ponto"""

  def P(self, t: float) -> tuple:
    """Retorna sempre o ponto `p`"""
    return self.__p

class Circle(Trajectory):
  """Classe que implementa uma trajetória circular"""
  def __init__(self, r: tuple, center: tuple):
    super().__init__()

    self.__center = center
    """Centro do círculo"""

    self.__radius = norm(r, center)
    """Raio"""

  def P(self, t: float) -> tuple:
    """Implementa um círculo que passa por tangente por `r` e tem raio `radius`"""
    x = self.__center[0] + self.__radius * np.cos(2*np.pi*t)
    y = self.__center[1] + self.__radius * np.sin(2*np.pi*t)
    o = 2*np.pi*t+np.pi/2
    return (x,y,o)

class UnifiedVectorField(Trajectory):
  """Classe que implementa uma trajetória UVF"""
  def __init__(self, source: str, r0: tuple, r: tuple, h=0.5, n=1, runUnified=True, showField=False):
    super().__init__()

    self.h = h
    self.n = n
    self.runUnified = runUnified
    self.showField = showField

    self.__r0 = np.array(r0)
    """Ponto inicial"""

    self.__r = np.array(r)
    """Ponto final"""

    self.__g = self.__r[:2] + self.h * self.unit(self.__r[2])
    """Ponto de referência"""

    self.__sampled = None
    """Trajetória amostrada"""

  def sample(self, samples=1000, dt=0.001, minDist=0.001, maxSamples=10000):
    sampled = []
    P = self.__r0[:2]
    while norm(P, self.__r) > minDist and len(sampled) < maxSamples:
      E = self.E(P)[:2]
      P = P+dt*E
      sampled.append((P[0], P[1], angl(E)))
    return sampled

  def unit(self, angle):
    return np.array([np.cos(angle), np.sin(angle)])

  def E(self, P):
    angle = ang(P, self.__r) - self.n * (ang(P, self.__g) - ang(P, self.__r))

    return np.array([*self.unit(angle), angle])

  def target(self, r0: tuple, d: float) -> tuple:
    """Retorna simplesmente `E` aplicado no pose do robô se a flag `runUnified` estiver habilitada, caso contrário, retorna um ponto da trajetória amostrada."""
    if self.runUnified:
      return (*r0[:2], self.E(r0)[2])
    else:
      return Trajectory.target(self, r0, d)

  def phi(self, pose: tuple, step: float, d=0.0001):
    """Calcula o ângulo \\(\\phi = \\frac{\\partial \\theta_d}{\\partial x} \\cos(\\theta) + \\frac{\\partial \\theta_d}{\\partial y} \\sin(\\theta)\\) usado para o controle"""
    pose = np.array(pose)
    dx = (self.E(pose+[d,0,0])[2]-self.E(pose)[2])/d
    dy = (self.E(pose+[0,d,0])[2]-self.E(pose)[2])/d
    return dx*np.cos(pose[2]) + dy*np.sin(pose[2])

  def P(self, t: float) -> tuple:
    if t <= 0: return self.__r0
    elif t >= 1: return self.__r

    if self.__sampled is None:
      self.__sampled = self.sample()

    index = int((t*(len(self.__sampled)-1)))

    return self.__sampled[index]

  def interestPoints(self):
    mapping = [(self.__g[0], self.__g[1], ang(self.__r, self.__g)), self.__r]
    if self.showField:
      for x in np.linspace(-1,1,40):
        for y in np.linspace(-1,1,40):
          E = self.E((x,y))
          mapping.append((x, y, angl(E)))
    return mapping
