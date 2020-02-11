from abc import ABC, abstractmethod
from controller.tools import ang, unit, angl, angError, norml, norm
import numpy as np

class GoalKeeperField():
  def __init__(self, Pb):
    self.y = Pb[1]
    self.Pb = Pb

  def F(self, P, Pb=None, retnparray=False):
    if len(P.shape) == 1: P = np.array([P]).T

    c0 = np.abs(P[0]-self.Pb[0]) > 0.07
    c1 = P[1] >= self.y
    c2 = np.bitwise_not(c1)

    #uvf = np.zeros_like(P[0])
    #uvf[c1] = -np.pi/2
    #uvf[c2] = np.pi/2
    uvf = ang(P, self.Pb)

    if uvf.size == 1 and not(retnparray): return uvf[0]
    return uvf

  def phi(self, P: tuple, d=0.00001):
    """Calcula o ângulo \\(\\phi = \\frac{\\partial \\theta_d}{\\partial x} \\cos(\\theta) + \\frac{\\partial \\theta_d}{\\partial y} \\sin(\\theta)\\) usado para o controle"""
    return 0
    
  def gamma(self, P: tuple, v: tuple, d=0.0001):
    return 0

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
    angle[c2] = 0

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
    return (dx*np.cos(P[2]) + dy*np.sin(P[2])) / 2

  def gamma(self, P: tuple, v: tuple, d=0.0001):
    P = np.array(P)
    Pb = np.array(self.Pb)
    return ((self.F(P, Pb=Pb+[d,0,0])-self.F(P))/d * v[0] +\
           (self.F(P, Pb=Pb+[0,d,0])-self.F(P))/d * v[1] +\
           (self.F(P, Pb=Pb+[0,0,d])-self.F(P))/d * v[2]) / 2

class UVFDefault(UVF):
  def __init__(self, world, pose, direction, radius=None):
    if radius is None: radius = world.getParam("UVF_r")
    
    super().__init__(pose,
      r=radius,
      Kr=world.getParam("UVF_Kr"),
      Kr_single=world.getParam("UVF_Kr_single"),
      direction=direction   
    )