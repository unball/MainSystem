from controller.control import SpeedPair, HLC
from controller.tools import sat, fixAngle
import numpy as np

class NLC(HLC):
  def __init__(self, source):
    super().__init__("Non Linear Control", source + "_NLC", {"ka": 1, "kg": 1, "kp": 1})
  
  def actuate(self, referencePose, robot, spin):
    # Computa os erros
    error_x = referencePose[0] - robot.x
    error_y = referencePose[1] - robot.y
    error_th = fixAngle(referencePose[2]-robot.th)
    
    # Obtém os parâmetros
    ka = self.getParam("ka")
    kg = self.getParam("kg")
    kp = self.getParam("kp")
    
    # Lei de controle da velocidade angular
    p = np.sqrt(error_x**2+error_y**2)
    gamma = np.arctan2(error_y, error_x)
    
    w = ka * error_th + kg * fixAngle(gamma-robot.th)
    w = sat(w, 4*np.pi)
    
    # Lei de controle da velocidade linear
    if robot.velmod < 0.01:
      sigma = robot.th
    else:
      sigma = robot.velang
    
    v = kp * p * np.cos(gamma-sigma) / np.cos(sigma-robot.th)
    
    return SpeedPair(v,w)
