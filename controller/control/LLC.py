from controller.control import SpeedPair, HLC
from controller.tools import norm, ang, angError, sat, bestAngError, fixAngle
import numpy as np
import math
import time

class UFC(HLC):
  """Controle unificado para o Univector Field, utiliza o ângulo definido pelo campo como referência \\(\\theta_d\\)."""
  def __init__(self, source):
    super().__init__("Univector Field Control", source + "_UFC", {"kw": 1, "kp": 10, "L": 0.075, "vmax": 0.7, "mu": 0.24, "motorangaccelmax": 999, "r": 0.03, "vref": 0, "maxangerror": 1, "tau": 0.1})

    self.g = 9.8

  def PImotorA(err):
    old_err = 0
    old_out = 0
    out = ( 1.6 * (err - 0.91  *  old_err) + old_out)
    old_err = err; #- (saturation(out)-out);

    if abs(out) < 255:
      old_out = out
    else:
      old_out = 0

    return out

  def PImotorB(err):
    """Implementa um PI digital com anti-windup para o motor A"""

    old_err = 0
    old_out = 0
    out = ( 1.6 * (err - 0.91  *  old_err) + old_out)
    old_err = err; #- (saturation(out)-out);

    if abs(out) < 255:
      old_out = out
    else:
      old_out = 0

    return out

  def actuate(self, currV, currW, v, w):

    # Obtém os parâmetros
    L = self.getParam("L")
    vmax = self.getParam("vmax")
    r = self.getParam("r")

    # Computa os erros de v e w
    eV = v - currV

    eW = w - currW
    
    # Erro nos controladores
    eA = eV + .2 * eW
    eB = eV - .2 * eW

    # Controle digital
    controlA = PImotorA(eA)
    controlB = PImotorB(eB)
    
    # Satura w caso ultrapasse a mudança máxima permitida
    #w  = lastspeed.w + sat(w-lastspeed.w, motorangaccelmax * r * interval / L)
    
    return v,w
