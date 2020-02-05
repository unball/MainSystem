from abc import ABC, abstractmethod
from controller.tools import ang, unit, angl, angError, norml, norm
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
  def __init__(self, world, Pb: tuple, repulsiveSize: float = 0.05, minDistance: float = 0.1):
    super().__init__()

    self.world = world
    self.Pb = Pb
    self.repulsiveSize = repulsiveSize
    self.minDistance = minDistance

  def d(self, P: tuple) -> float:
    return np.exp(-(self.world.ymax-abs(P[1]))**2/self.minDistance**2)
    
  def F(self, P: tuple) -> float:
    return (np.arctan((self.Pb[0]-P[0])/self.repulsiveSize)-np.pi/2) * np.sign(P[1])
    #return -(np.arctan((P[0]-self.Pb[0])/self.repulsiveSize)+np.pi/2)/2 * np.sign(P[1])

class VerticalObstacle(Obstacle):
  def __init__(self, world, Pb: tuple, avoidGoal=True, repulsiveSize: float = 0.15, goalSize: float = 0.2, minDistance: float = 0.15):
    super().__init__()

    self.world = world
    self.Pb = Pb
    self.avoidGoal = avoidGoal
    self.repulsiveSize = repulsiveSize
    self.goalSize = goalSize
    self.minDistance = minDistance

  def d(self, P: tuple) -> float:
    return np.exp(-(self.world.xmax-abs(P[0]))**2/self.minDistance**2) * (1-(P[0] > 0) * np.exp(-(P[1])**2/self.goalSize**2)) if not self.avoidGoal else np.exp(-(self.world.xmax-P[0])**2/self.minDistance**2)
    
  def F(self, P: tuple) -> float:
    return -np.arctan((self.Pb[1]-P[1])/self.repulsiveSize)*np.sign(P[0]) - np.pi * (P[0] > 0)

class PointObstacle(Obstacle):
  def __init__(self, world, Pb: tuple, Po: tuple, radius: float = 0.3, distanceRadius: float = 0.1, minAvoidanceAngle: float = 0.5):
    super().__init__()

    self.world = world
    self.Pb = Pb
    self.Po = Po
    self.radius = radius
    self.distanceRadius = distanceRadius
    self.minAvoidanceAngle = minAvoidanceAngle

  def d(self, P: tuple) -> float:
    theta = angError(angl((P[:2].T-np.array(self.Po[:2])).T), angl(np.array(self.Pb[:2])-np.array(self.Po[:2])))
    return np.exp(-((P[0]-self.Po[0])**2+(P[1]-self.Po[1])**2)/self.distanceRadius**2) * (1-np.exp(-(theta/self.minAvoidanceAngle)**2))
    
  def F(self, P: tuple) -> float:
    ret = np.array(P[0])*0
    filterCCW = angError(ang(P, self.Po), ang(self.Po, self.Pb)) >= 0
    filterCW = np.logical_not(filterCCW)
    ret[filterCCW] = self.CCW(P[0][filterCCW]-self.Po[0], P[1][filterCCW]-self.Po[1])
    ret[filterCW] = self.CW(P[0][filterCW]-self.Po[0], P[1][filterCW]-self.Po[1])
    return ret

  def CCW(self, x: float, y: float) -> float:
    return np.arctan2((x+y*(self.radius**2-x**2-y**2)), (-y+x*(self.radius**2-x**2-y**2)))

  def CW(self, x: float, y: float) -> float:
    return np.arctan2((-x+y*(self.radius**2-x**2-y**2)), (y+x*(self.radius**2-x**2-y**2)))

# class UVF:
#   """Classe que implementa um campo UVF"""
#   def __init__(self, Pb: tuple, world, rg: tuple = None, h: float=0.5, n: float=1, avoidGoal=True, pointObstacles=[], 
#     horRepSize: float = 0.05, 
#     horMinDist: float = 0.1, 
#     verRepSize: float = 0.15, 
#     verGoalSize: float = 0.2, 
#     verMinDist: float = 0.15, 
#     ponRadius: float = 0.3, 
#     ponDistanceRadius: float = 0.1, 
#     ponMinAvoidanceAngle: float = 0.5
#   ):

#     self.rg = rg

#     self.h = h
#     """Distância do ponto guia"""
    
#     self.n = n
#     """Parâmetro `n`"""
    
#     self.world = world
#     """Referência ao mundo"""
    
#     self.Pb = (Pb[0], Pb[1], Pb[2])
#     """Ponto final"""

#     self.obstacles = [
#       #HorizontalObstacle(world, Pb, horRepSize, horMinDist),
#       #VerticalObstacle(world, Pb, False, verRepSize, verGoalSize, verMinDist)
#     ]

#     for pointObstacle in pointObstacles: self.obstacles.append(PointObstacle(world, Pb, pointObstacle, ponRadius, ponDistanceRadius, ponMinAvoidanceAngle))

#   def F(self, P: tuple, Pb=None, retnparray=False):
#     """Calcula o campo no ponto `P`. Recebendo `Pb`, a posição do target final"""
#     if len(P.shape) == 1: P = np.array([P]).T
    
#     if Pb is None: Pb = self.Pb
    
#     # Calcula o campo UVF de chegada
#     targetAngle = ang(P, Pb) - self.n * (ang(P, self.Pg(Pb)) - ang(P, Pb))
    
#     # Calcula o campo UVF de desvio de obstáculos
#     avoidanceVector = [0,0]
#     avoidanceWeights = []
#     for obstacle in self.obstacles:
#       fo = obstacle.d(P)
#       u = unit(obstacle.F(P))
#       avoidanceVector[0] += fo * u[0]
#       avoidanceVector[1] += fo * u[1]
#       avoidanceWeights.append(fo)

#     avoidanceAngle = angl(avoidanceVector)
    
#     # Une os campos
#     if len(avoidanceWeights) > 0:
#       f = np.max(np.array(avoidanceWeights).T,axis=1)
#       angle = angl(unit(targetAngle) * (1-f) + f * unit(avoidanceAngle))
#     else:
#       angle = targetAngle

#     if angle.size == 1 and not(retnparray): return angle[0]
#     return angle
    
#   def Pg(self, Pb: tuple):
#     """Retorna o ponto guia"""
#     return Pb[:2] + self.h * unit(Pb[2]) if self.rg is None else self.rg

#   def phi(self, P: tuple, d=0.0001):
#     """Calcula o ângulo \\(\\phi = \\frac{\\partial \\theta_d}{\\partial x} \\cos(\\theta) + \\frac{\\partial \\theta_d}{\\partial y} \\sin(\\theta)\\) usado para o controle"""
#     P = np.array(P)
#     dx = (self.F(P+[d,0,0])-self.F(P))/d
#     dy = (self.F(P+[0,d,0])-self.F(P))/d
#     return dx*np.cos(P[2]) + dy*np.sin(P[2])

#   def gamma(self, P: tuple, d=0.0001):
#     P = np.array(P)
#     Pb = np.array(self.Pb)
#     return (self.F(P, Pb=Pb+[d,0,0])-self.F(P))/d * self.world.ball.inst_vx +\
#            (self.F(P, Pb=Pb+[0,d,0])-self.F(P))/d * self.world.ball.inst_vy

class UVF():
  def __init__(self, Pb, r, Kr, Kr_single, direction=0):
    self.r = r
    self.Kr = Kr
    self.Kr_single = Kr_single
    self.Pb = Pb
    self.direction = direction

  def F(self, P, Pb=None, retnparray=False):
    if len(P.shape) == 1: P = np.array([P]).T

    if Pb is None: Pb = self.Pb

    # Ajusta o sistema de coordenadas
    Pb = np.array([Pb])
    P[:2] = P[:2]-Pb.T[:2]
    rotMatrix = np.array([[np.cos(Pb[0][2]), np.sin(Pb[0][2])],[-np.sin(Pb[0][2]), np.cos(Pb[0][2])]])
    P[:2] = np.matmul(rotMatrix, P[:2])

    # Campo UVF
    uvf = np.zeros(P[0].size)

    # Regiões do campo
    c1 = np.abs(P[1]) <= self.r
    c2 = P[1] < -self.r
    c3 = P[1] > self.r
    
    # Peso das espirais
    yl = -P[1][c1] + self.r
    yr = +P[1][c1] + self.r

    # Centros das espirais
    Pl = np.array([P[0], P[1]-self.r])
    Pr = np.array([P[0], P[1]+self.r])

    # Gera cada região
    if self.direction == 0:
      uvf[c1] = angl((yl*self.N(Pr.T[c1].T, -1) + yr*self.N(Pl.T[c1].T, +1)) / (2*self.r))
      uvf[c2] = angl(self.N(Pr.T[c2].T, -1))
      uvf[c3] = angl(self.N(Pl.T[c3].T, +1))
    elif self.direction == 1:
      uvf = angl(self.M(Pl, +1, self.r, self.Kr_single))
    elif self.direction == -1:
      uvf = angl(self.M(Pr, -1, self.r, self.Kr_single))

    # Ajusta o sistema de coordenadas
    uvf = uvf + Pb[0][2]

    if uvf.size == 1 and not(retnparray): return uvf[0]
    return uvf
  
  def alpha(self, P, sign, r, Kr):
    c1 = norml(P) > r
    c2 = np.logical_not(c1)
    angle = np.zeros(P[0].size)

    angle[c1] = angl(P.T[c1].T) + sign * np.pi/2 * (2 - (r+Kr) / (norml(P.T[c1].T) + Kr))
    angle[c2] = ang(P.T[c2].T, (0,-sign*r)) #angl(P.T[c2].T) + sign * np.pi/2 * np.sqrt(norml(P.T[c2].T) / r)

    return angle

  def N(self, P, sign):
    return unit(self.alpha(P, sign, self.r, self.Kr))

  def M(self, P, sign, r, Kr):
    return unit(self.alpha(P, sign, r, Kr))

  def phi(self, P: tuple, d=0.00001):
    """Calcula o ângulo \\(\\phi = \\frac{\\partial \\theta_d}{\\partial x} \\cos(\\theta) + \\frac{\\partial \\theta_d}{\\partial y} \\sin(\\theta)\\) usado para o controle"""
    P = np.array(P)
    dx = (self.F(P+[d,0,0])-self.F(P))/d
    dy = (self.F(P+[0,d,0])-self.F(P))/d
    return dx*np.cos(P[2]) + dy*np.sin(P[2])

  def gamma(self, P: tuple, vx, vy, d=0.0001):
    P = np.array(P)
    Pb = np.array(self.Pb)
    return (self.F(P, Pb=Pb+[d,0,0])-self.F(P))/d * vx +\
           (self.F(P, Pb=Pb+[0,d,0])-self.F(P))/d * vy

class UVFDefault(UVF):
  def __init__(self, world, pose, direction):
    super().__init__(pose,
      r=world.getParam("UVF_r"),
      Kr=world.getParam("UVF_Kr"),
      Kr_single=world.getParam("UVF_Kr_single"),
      direction=direction   
    )