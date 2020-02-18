from abc import ABC, abstractmethod
from controller.tools import ang, unit, angl, angError, norml, norm, sat, filt, insideRect
import numpy as np

class Field(ABC):
  def __init__(self, Pb):
    super().__init__()
    self.Pb = Pb

  @abstractmethod
  def F(self, P, Pb=None, retnparray=False):
    pass

  def phi(self, P: tuple, d=0.00001):
    """Calcula o ângulo \\(\\phi = \\frac{\\partial \\theta_d}{\\partial x} \\cos(\\theta) + \\frac{\\partial \\theta_d}{\\partial y} \\sin(\\theta)\\) usado para o controle"""
    P = np.array(P)
    dx = filt((self.F(P+[d,0,0])-self.F(P))/d, 100)
    dy = filt((self.F(P+[0,d,0])-self.F(P))/d, 100)
    return (dx*np.cos(P[2]) + dy*np.sin(P[2])) / 2

  def gamma(self, P: tuple, v: tuple, d=0.00001):
    P = np.array(P)
    Pb = np.array(self.Pb)
    dx = filt((self.F(P, Pb=Pb+[d,0,0])-self.F(P))/d, 100)
    dy = filt((self.F(P, Pb=Pb+[0,d,0])-self.F(P))/d, 100)
    dth = filt((self.F(P, Pb=Pb+[0,0,d])-self.F(P))/d, 100)
    return (dx * v[0] + dy * v[1] + dth * v[2]) / 2

  
class DefenderField(Field):
  def __init__(self, Pb, a=0.3, b=0.45, center=[0.75, 0]):
    super().__init__(Pb)
    self.a = a
    self.b = b
    self.center = np.array(center)

  def F(self, P, Pb=None, retnparray=False):
    if len(P.shape) == 1: P = np.array([P]).T

    P[:2] = (P[:2].T - self.center).T

    d = np.abs(np.sqrt(P[0]**2 / self.a ** 2 + P[1]**2 / self.b ** 2) - 1) > 0.5

    x1 = P[0] / self.a
    x2 = P[1] / self.b

    ccw = angError(angl(self.Pb[:2]-self.center), angl(P)) > 0
    cw = np.bitwise_not(ccw)
    uvf = np.zeros_like(P[0])

    uvf[ccw] = np.arctan2(x1 + x2 * (1 - x1**2 - x2**2), -x2 + x1 * (1 - x1**2 - x2**2))[ccw]
    uvf[cw] = np.arctan2(-x1 + x2 * (1 - x1**2 - x2**2), x2 + x1 * (1 - x1**2 - x2**2))[cw]

    # uvf = np.arctan2(- self.b**2 * P[0], self.a**2 * P[1])
    # uvf[ccw] = uvf[ccw] + np.pi

    uvf[d] = ang((P[:2].T[d] + self.center).T, self.Pb)

    if uvf.size == 1 and not(retnparray): return uvf[0]
    return uvf

  def phi(self, P: tuple, d=0.00001):
    """Calcula o ângulo \\(\\phi = \\frac{\\partial \\theta_d}{\\partial x} \\cos(\\theta) + \\frac{\\partial \\theta_d}{\\partial y} \\sin(\\theta)\\) usado para o controle"""
    return 0
    
  def gamma(self, P: tuple, v: tuple, d=0.0001):
    return 0

class GoalKeeperField(Field):
  def __init__(self, Pb):
    super().__init__(Pb)
    self.y = Pb[1]

  def F(self, P, Pb=None, retnparray=False):
    if len(P.shape) == 1: P = np.array([P]).T

    c0 = np.abs(P[0]-self.Pb[0]) > 0.07
    c1 = P[1] >= self.y
    c2 = np.bitwise_not(c1)

    uvf = np.zeros_like(P[0])
    uvf[c1] = -np.pi/2
    uvf[c2] = np.pi/2
    #uvf = ang(P, self.Pb)

    if uvf.size == 1 and not(retnparray): return uvf[0]
    return uvf
    
  def gamma(self, P: tuple, v: tuple, d=0.0001):
    return 0

class UVF(Field):
  def __init__(self, Pb, Pr, r, Kr, Kr_single, direction=0, spiral=True):
    super().__init__(Pb)
    self.r = r
    self.Kr = Kr
    self.Kr_single = Kr_single
    self.Ko = 0.12
    self.dmin = 0.0348
    self.delta = 0.0457
    self.Pr = Pr
    self.direction = direction
    self.spiral = spiral

  def TUF(self, P, Pb=None):
    P = P.copy()

    # Permite uso de Pb alternativo
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
    return uvf + Pb[0][2]

  def AUF(self, P, Pr, Vr, Po, Vo):
    # Vetor de deslocamento
    s = self.Ko * (Vo-Vr)

    # Obstáculo virtual
    d = norm(Pr, Po)
    if d >= norml(s):
      Pvo = Po + s
    else:
      Pvo = Po + (d / norml(s)) * s

    return ang(Pvo, P), norm(Pvo, P)

  def F(self, P, Pb=None, retnparray=False):
    if len(P.shape) == 1: P = np.array([P]).T

    uvf = self.th(P, Pb)

    if uvf.size == 1 and not(retnparray): return uvf[0]
    return uvf

  def th(self, P, Pb):
    tuf = self.TUF(P, Pb=Pb)
    # auf, R = self.AUF(P, self.Pr, np.array([0,0]), np.array([0,0]), np.array([0,0]))

    # c1 = R <= self.dmin
    # c2 = np.bitwise_not(c1)

    # uvf = np.zeros_like(P[0])
    # uvf[c1] = auf[c1]
    # uvf[c2] = auf[c2] * self.G(R[c2]-self.dmin, self.delta) + tuf[c2] * (1-self.G(R[c2]-self.dmin, self.delta))
    return tuf

  def G(self, x, delta):
    return np.exp(-(x/delta)**2/2)
  
  def alpha(self, P, sign, r, Kr):
    #r2 = r/2
    c1 = norml(P) >= r
    #c2 = np.bitwise_and(norml(P) < r, norml(P) >= r2)
    c2 = np.bitwise_not(c1)
    #c3 = norml(P) < r2
    angle = np.zeros(P[0].size)

    angle[c1] = angl(P.T[c1].T) + sign * np.pi/2 * (2 - (r+Kr) / (norml(P.T[c1].T) + Kr))
    # #angle[c2] = 0#angl(P.T[c2].T) + sign * np.pi/2 * np.sqrt(norml(P.T[c2].T) / r) #ang(P.T[c2].T, (0,-sign*r))
    # x = P.T[c2].T[0]
    # y = P.T[c2].T[1]
    # angle[c2] = np.arctan2(x*sign + 15 *(r-x**2) * y, -y*sign)
    # angle[c3] = 0#np.arctan2(x*sign + 15 *(r-x**2) * y, -y*sign)
    if self.spiral:
      angle[c2] = angl(P.T[c2].T) + sign * np.pi/2 * np.sqrt(norml(P.T[c2].T) / r)
    else:
       angle[c2] = 0

    return angle

  def N(self, P, sign):
    return unit(self.alpha(P, sign, self.r, self.Kr))

  def M(self, P, sign, r, Kr):
    return unit(self.alpha(P, sign, r, Kr))

class UVFDefault(UVF):
  def __init__(self, world, pose, robotPose, direction, radius=None, spiral=True):
    if radius is None: radius = world.getParam("UVF_r")
    
    super().__init__(pose, robotPose,
      r=radius,
      Kr=world.getParam("UVF_Kr"),
      Kr_single=world.getParam("UVF_Kr_single"),
      direction=direction, spiral = spiral
    )

class UVFavoidGoalArea(UVF):
  def __init__(self, world, pose, robotPose, radius=None):
    if radius is None: radius = world.getParam("UVF_r")
    self.rm = world.allyGoalPos
    self.s = world.goalAreaSize + [0.05,0.05]
    super().__init__(pose, robotPose,
      r=radius,
      Kr=world.getParam("UVF_Kr"),
      Kr_single=world.getParam("UVF_Kr_single"),
      direction=0
    )
  
  def F(self, P, Pb=None, retnparray=False):
    if len(P.shape) == 1: P = np.array([P]).T

    c1 = np.bitwise_and(np.abs(P[0]-self.rm[0]) < self.s[0], np.abs(P[1]-self.rm[1]) < self.s[1])
    c2 = np.bitwise_not(c1)
    uvf = np.zeros_like(P[0])
    uvf[c1] = 0
    uvf[c2] = super().th(P.T[c2].T,Pb)

    if uvf.size == 1 and not(retnparray): return uvf[0]
    return uvf