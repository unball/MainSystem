from abc import ABC, abstractmethod
import scipy.optimize
import numpy as np
import matplotlib.pyplot as mlp
import math
import time
import dubins

class Trajectory(ABC):
  def __init__(self):
    super().__init__()
  
  @abstractmethod
  def P(self, t: float) -> tuple:
    """Função de parametrização da trajetória. Deve ser: \\(\\vec{P}(t) = (\\vec{r}(t), \\theta(t))\\), em que \\(\\|\\vec{r}(t)-\\vec{r}_0\\|\\) é contínua para todo \\(\\vec{r}_0\\) e \\(t \\in [0,1]\\)"""
    pass
  
  def norm(self, ra: tuple, rb: tuple) -> float:
    """Calcula  \\(\\|\\vec{r}_a-\\vec{r}_b\\|\\)"""
    return math.sqrt((ra[0]-rb[0])**2 + (ra[1]-rb[1])**2)
  
  def tofindmin(self, t: float, r0: tuple) -> float:
    """Calcula  \\(\\|\\vec{r}(t)-\\vec{r}_0\\|\\)"""
    return self.norm(self.P(t), r0)
  
  def tofindroot(self, t: float, r0: tuple, d: float) -> float:
    """Calcula  \\(\\|\\vec{r}(t)-\\vec{r}_0\\| - d\\)"""
    return self.norm(self.P(t), r0) - d
    
  def tmin(self, r0: tuple):
    return scipy.optimize.brute(self.tofindmin, [(0,1)], args=(r0,))[0]
  
  def target(self, r0: tuple, d: float) -> tuple:
    """Função que retorna o target que dista `d` do ponto `r0` pertence à trajetória no sentido de progressão do percurso. Faz isso encontrando \\(t_{opt}\\) tal que: \\(\\|\\vec{r}(t_{opt})-\\vec{r}_0\\| - d = 0\\) usando o método de Brent e \\(t \\in [t_{min}, 1]\\) e retorna \\(\\vec{P}(t_{opt})\\)"""
    tmin = self.tmin(r0)
    dmin = self.tofindmin(tmin, r0)
    
    try:
      t = scipy.optimize.brentq(self.tofindroot, tmin, 1, args=(r0,max(dmin,d)))
    except:
      t = 1
    
    return self.P(t)
  
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
