import numpy as np

# Constantes físicas do robô
wheel_reduction = 5
r = 0.0325
L = 0.075
#L = 0.18

def speeds2motors(v: float, w: float) -> (int, int):
  """Recebe velocidade linear e angular e retorna velocidades para as duas rodas"""

  # Computa a velocidade angular de rotação de cada roda
  vr = (v + (L/2)*w) / r#/ (2*np.pi*r) * wheel_reduction
  vl = (v - (L/2)*w) / r#/ (2*np.pi*r) * wheel_reduction

  #if fabs(vr) > max_motor_speed or fabs(vl) > max_motor_speed:
  #  vr = max_motor_speed * vr / max(vr, vl)
  #  vl = max_motor_speed * vl / max(vr, vl)
  
  # vr *= convertion
  # vl *= convertion
  
  return vl, vr

def motors2linvel(vl: float, vr: float) -> float:
  # Computa a velocidade angular de rotação de cada roda
  return (vr + vl) * (2*np.pi*r) / wheel_reduction / 2

def angl(p0: tuple):
  """Calcula o ângulo da tupla `p0` no plano este ângulo está em \\((-\\pi,\\pi]\\)"""
  return np.arctan2(p0[1], p0[0])

def unit(angle):
  """Retorna um vetor unitário de ângulo `angle` no formato de numpy array"""
  return np.array([np.cos(angle), np.sin(angle)])

def norm(p0: tuple, p1: tuple):
  """Calcula a distância entre as tuplas `p0` e `p1` no plano"""
  return np.sqrt((p1[0]-p0[0])**2+(p1[1]-p0[1])**2)

def norml(p0: tuple):
  """Calcula a norma de `p0"""
  return np.sqrt((p0[0])**2+(p0[1])**2)

def angError(reference: float, current: float) -> float:
  """Calcula o erro angular entre `reference` e `current` de modo que este erro esteja no intervalo de \\((-\\pi,\\pi]\\) e o sinal do erro indique qual deve ser a orientação para seguir a referência, de modo que positivo é anti-horário e negativo é horário"""
  diff = np.arccos(np.cos(reference-current))
  sign = (np.sin(reference-current) >= 0)*2-1
  return sign * diff

def adjustAngle(angle: float) -> float:
  """Pega um ângulo em \\(\\mathbb{R}\\) e leva para o correspondente em \\((-\\pi,\\pi]\\)"""
  return angError(angle, 0)

def sat(x: float, amp: float):
  """Satura um número real `x` entre `amp` e `-amp`"""
  return max(min(x, amp), -amp)

def ang(p0: tuple, p1: tuple):
  """Calcula o ângulo entre as tuplas `p0` e `p1` no plano, este ângulo está em \\((-\\pi,\\pi]\\)"""
  return np.arctan2(p1[1]-p0[1], p1[0]-p0[0])

def filt(x: float, amp: float):
  if np.abs(x) > np.abs(amp): return 0
  else: return x

def fixAngle(angle: float):
  if abs(angle) > np.pi/2:
    return (angle + np.pi/2) % (np.pi) - np.pi/2
  else:
    return angle