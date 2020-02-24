from controller.world.element import Element
from controller.control.UFC import UFC
from controller.control import SpeedPair
from controller.tools import adjustAngle, angError
import numpy as np
import time

class Robot(Element):
  """Classe filha que implementa um robô no campo."""

  def __init__(self, world, worldIdx, controlSystem=UFC("defaultRobot")):
    super().__init__(world)
    
    self.step = 0.03
    """Distância ao próximo target da trajetória"""
    
    self.__field = None
    """Campo definido para este robô"""
    
    self.__controlSystem = controlSystem
    """Sistema de controle para este robô"""
    
    self.dir = 1
    """Direção de movimento do robô, 1 é para frente e -1 é para trás"""

    self.entity = None
    """Entidade do robô, pode ser Attacker, Goalkeeper, Defender, ..."""

    self.lastAngError = 0

    self.lastControlLinVel = 0

    self.vref = 0

    self.gammavels = (0,0,0)

    self.lastTimeAlive = None

    """Ativa o spin, onde '1' é horário e '-1' é """
    self.spin = 0

    self.preferedEntity = ""

    self.size = 0.080

    self.spinTime = 0

    self.meanId = worldIdx

  def actuate(self):
    """Retorna velocidade linear e angular de acordo com o controle do robô e o campo utilizado por ele"""
    if self.field is None: return SpeedPair(0,0)

    reference = self.field.F(self.pose)

    v,w = self.controlSystem.actuate(reference, self.pose, self.field, self.dir, self.gammavels, self.vref, self.spin)
    self.lastControlLinVel = v
    self.lastAngError = angError(reference, self.th)

    return SpeedPair(v * self.dir, -w * self.world.fieldSide)

  @property
  def field(self):
    """Retorna o campo definida para este robô"""
    return self.__field

  @field.setter
  def field(self, field):
    """Atualiza o campo definido para este robô"""
    self.__field = field

  @property
  def controlSystem(self):
    """Retorna o sistema de controle do robô"""
    return self.__controlSystem

  @controlSystem.setter
  def controlSystem(self, controlSystem):
    """Atualiza o sistema de controle do robô"""
    self.__controlSystem = controlSystem

  def calc_velocities(self, dt, alpha=0.5, thalpha=0.8):
    super().calc_velocities(dt, alpha=0.8)

  @property
  def th(self):
    """Redefinição de th que leva em consideração a direção do robô."""
    return adjustAngle(super().th + (np.pi if self.dir == -1 else 0))

  @property
  def dir_raw_th(self):
    return adjustAngle(self.inst_th + (np.pi if self.dir == -1 else 0))

  def dir_raw_update(self, x=0, y=0, th=0):
    """Atualiza a posição do objeto, atualizando também o valor das posições anteriores."""
    super().raw_update(x, y, th - (np.pi if self.dir == -1 else 0))

  @th.setter
  def th(self, th):
    """Atualiza o ângulo do objeto diretamente (sem afetar o ângulo anterior)."""
    self.setTh(th - (np.pi if self.dir == -1 else 0))

  def isAlive(self):
    """Verifica se o robô está vivo baseado na relação entre a velocidade enviada pelo controle e a velocidade medida pela visão"""
    ctrlVel = np.abs(self.lastControlLinVel)

    if ctrlVel < 0.01 or not self.world.running:
      self.lastTimeAlive = time.time()
      return True

    if self.velmod / ctrlVel < 0.15:
      if self.lastTimeAlive is not None and time.time()-self.lastTimeAlive > 1:
        return False
    else:
      self.lastTimeAlive = time.time()
    
    return True
    
