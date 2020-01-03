from controller.control import SpeedPair, HLC
from controller.tools import fixAngle, norm, ang
import numpy as np
import math


def ctrlLaw(referenceAngle, robot, spin, kw, kp, L, amax, vmax):
  """Lei de controle baseada no livro Springer Tracts in Advanced Robotics - Soccer Robotics para o controle unificado. Esta função implementa a lei:
  $$
  v = \\min(v_1,v_2,v_3)\\\\
  v_1 = \\frac{-K_{\\omega} \\sqrt{|\\theta_e|} + \\sqrt{K_{\\omega}^2 |\\theta_e| + 4 |\\phi| A_m}}{2|\\phi|}\\\\
  v_2 = \\frac{2 \\cdot V_m - L \\cdot K_{\\omega} \\sqrt{|\\theta_e|} }{2+L \\cdot |\\phi|}\\\\
  v_3 = K_p ||\\vec{r}-\\vec{P}(1)||\\\\
  \\phi = \\frac{\\partial \\theta_d}{\\partial x} \\cos(\\theta) + \\frac{\\partial \\theta_d}{\\partial y} \\sin(\\theta)\\\\
  \\omega = v \\cdot \\phi + K_{\\omega} \\cdot \\text{sign}(\\theta_e) \\sqrt{|\\theta_e|}
  $$"""
  # Computa os erros
  errorAngle = fixAngle(referenceAngle-robot.th)

  # Se a trajetória for incapaz de calcular o phi, então usa 0
  if getattr(robot.trajectory, "phi", None) is None:
    phi = 0
    v1 = math.inf if errorAngle == 0 else amax / (kw * np.sqrt(np.abs(errorAngle)))

  # Calcula o phi
  else:
    phi = robot.trajectory.phi(robot.pose, robot.step)
    v1 = (-kw * np.sqrt(np.abs(errorAngle)) + np.sqrt(kw**2 * np.abs(errorAngle) + 4 * np.abs(phi) * amax)) / (2*np.abs(phi))

  # Lei de controle da velocidade linear
  v2 = (2*vmax - L * kw * np.sqrt(np.abs(errorAngle))) / (2 + L * np.abs(phi))
  v3 = kp * norm(robot.pos, robot.trajectory.P(1))
  v  = min(v1, v2, v3)

  # Lei de controle da velocidade angular
  w = v * phi + kw * np.sign(errorAngle) * np.sqrt(np.abs(errorAngle))

  return SpeedPair(v,w)

class UFC(HLC):
  """Controle unificado para o Univector Field, utiliza o ângulo definido pelo campo como referência \\(\\theta_d\\)."""
  def __init__(self, source):
    super().__init__("Univector Field Control", source + "_UFC", {"kw": 1, "kp": 1, "L": 0.075, "vmax": 1, "mu": 1})

    self.g = 9.8

  def actuate(self, referencePose, robot, spin):

    # Obtém os parâmetros
    kw = self.getParam("kw")
    kp = self.getParam("kp")
    L = self.getParam("L")
    amax = self.getParam("mu") * self.g
    vmax = self.getParam("vmax")

    return ctrlLaw(referencePose[2], robot, spin, kw, kp, L, amax, vmax)

class NLCUFC(HLC):
  """Controle baseado no UFC, mas utiliza como ângulo de referência o ângulo entre a posição do robô e o target"""
  def __init__(self, source):
    super().__init__("Non-Linear Control (baseado no UFC)", source + "_NLCUFC", {"kw": 1, "kp": 1, "L": 0.075, "vmax": 1, "mu": 1})

    self.g = 9.8

  def actuate(self, referencePose, robot, spin):

    # Obtém os parâmetros
    kw = self.getParam("kw")
    kp = self.getParam("kp")
    L = self.getParam("L")
    amax = self.getParam("mu") * self.g
    vmax = self.getParam("vmax")

    # Ângulo de referência é o ângulo entre o robô e o ponto da trajetória
    referenceAngle = ang(robot.pose, referencePose)

    return ctrlLaw(referenceAngle, robot, spin, kw, kp, L, amax, vmax)
