import numpy as np

def norm(p0: tuple, p1: tuple):
  """Calcula a distância entre as tuplas `p0` e `p1` no plano"""
  return np.sqrt((p1[0]-p0[0])**2+(p1[1]-p0[1])**2)

def ang(p0: tuple, p1: tuple):
  """Calcula o ângulo entre as tuplas `p0` e `p1` no plano, este ângulo está em \\((-\\pi,\\pi]\\)"""
  return np.arctan2(p1[1]-p0[1], p1[0]-p0[0])

def angl(p0: tuple):
  """Calcula o ângulo da tupla `p0` no plano este ângulo está em \\((-\\pi,\\pi]\\)"""
  return np.arctan2(p0[1], p0[0])

def sat(x: float, amp: float):
  """Satura um número real `x` entre `amp` e `-amp`"""
  return max(min(x, amp), -amp)

def fixAngle(angle: float):
  if abs(angle) > np.pi/2:
    return (angle + np.pi/2) % (np.pi) - np.pi/2
  else:
    return angle
