from controller.control import SpeedPair, HLC
from controller.tools import norm, ang, angError, sat, bestAngError, fixAngle
import numpy as np
import math
import time

class UFC(HLC):
  """Controle unificado para o Univector Field, utiliza o ângulo definido pelo campo como referência \\(\\theta_d\\)."""
  def __init__(self, source):
    super().__init__("Univector Field Control", source + "_UFC", {"kw": 1, "kp": 10, "L": 0.075, "vmax": 0.7, "mu": 0.24, "motorangaccelmax": 999, "r": 0.03, "tau": 0.100, "vref": 0})

    self.g = 9.8

  def actuate(self, reference, currentPose, field):
    """Lei de controle baseada no livro Springer Tracts in Advanced Robotics - Soccer Robotics para o controle unificado. Esta função implementa a lei:
    $$
    v = \\min(v_1,v_2,v_3)\\\\
    v_1 = \\frac{-K_{\\omega} \\sqrt{|\\theta_e|} + \\sqrt{K_{\\omega}^2 |\\theta_e| + 4 |\\phi| A_m}}{2|\\phi|}\\\\
    v_2 = \\frac{2 \\cdot V_m - L \\cdot K_{\\omega} \\sqrt{|\\theta_e|} }{2+L \\cdot |\\phi|}\\\\
    v_3 = K_p ||\\vec{r}-\\vec{P}(1)||\\\\
    \\phi = \\frac{\\partial \\theta_d}{\\partial x} \\cos(\\theta) + \\frac{\\partial \\theta_d}{\\partial y} \\sin(\\theta)\\\\
    \\omega = v \\cdot \\phi + K_{\\omega} \\cdot \\text{sign}(\\theta_e) \\sqrt{|\\theta_e|}
    $$"""

    # Obtém os parâmetros
    kw = self.getParam("kw")
    kp = self.getParam("kp")
    L = self.getParam("L")
    amax = self.getParam("mu") * self.g
    vmax = self.getParam("vmax")
    motorangaccelmax = self.getParam("motorangaccelmax")
    r = self.getParam("r")
    tau = self.getParam("tau")
    vref = self.getParam("vref")
      
    # Computa os erros
    errorAngle = angError(reference, currentPose[2])

    # Velocidade limite de deslizamento
    phi = field.phi(currentPose)
    if phi != 0:
      v1 = (-kw * np.sqrt(np.abs(errorAngle)) + np.sqrt(kw**2 * np.abs(errorAngle) + 4 * np.abs(phi) * amax)) / (2*np.abs(phi))
    else:
      v1 = amax / (kw * np.sqrt(np.abs(errorAngle)))

    # Velocidade limite das rodas
    v2 = (2*vmax - L * kw * np.sqrt(np.abs(errorAngle))) / (2 + L * np.abs(phi))

    # Velocidade limite de aproximação
    v3 = kp * norm(currentPose, field.Pb) ** 2 + vref
    
    v  = min(v1, v2, v3)
    
    # Satura v caso ultrapasse a mudança máxima permitida
    #if v > lastspeed.v: v = lastspeed.v + sat(v-lastspeed.v, motorangaccelmax * r * interval / 2)

    # Lei de controle da velocidade angular
    w = v * phi + field.gama(currentPose) + kw * np.sign(errorAngle) * np.sqrt(np.abs(errorAngle))
    
    # Satura w caso ultrapasse a mudança máxima permitida
    #w  = lastspeed.w + sat(w-lastspeed.w, motorangaccelmax * r * interval / L)
    
    return SpeedPair(v,w)
