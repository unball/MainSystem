from controller.control import SpeedPair, HLC
from controller.tools import fixAngle, norm
import numpy as np

class UFC(HLC):
  def __init__(self, source):
    super().__init__("Univector Field Control", source + "_UFC", {"kw": 1, "kp": 1, "amax": 1, "L": 1, "vmax": 1})

  def actuate(self, referencePose, robot, spin):
    # Computa os erros
    errorAngle = fixAngle(referencePose[2]-robot.th)

    # Obtém os parâmetros
    kw = self.getParam("kw")
    kp = self.getParam("kp")
    L = self.getParam("L")
    amax = self.getParam("amax")
    vmax = self.getParam("vmax")

    # Só continua se for uma trajetória capaz de computar o phi
    if getattr(robot.trajectory, "phi", None) is None: return SpeedPair(0,0)

    # Calcula o phi
    phi = robot.trajectory.phi(robot.pose)

    # Lei de controle da velocidade linear
    v1 = (-kw * np.sqrt(np.abs(errorAngle)) + np.sqrt(kw**2 * np.abs(errorAngle) + 4 * np.abs(phi) * amax)) / (2*np.abs(phi))
    v2 = (2*vmax - L * kw * np.sqrt(np.abs(errorAngle))) / (2 + L * np.abs(phi))
    v3 = kp * norm(robot.pos, robot.trajectory.P(1))
    v  = min(v1, v2, v3)

    # Lei de controle da velocidade angular
    w = v * phi + kw * np.sign(errorAngle) * np.sqrt(np.abs(errorAngle))

    return SpeedPair(v,w)
