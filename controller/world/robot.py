from controller.world.element import Element
from controller.control.UFC import UFC
from controller.tools import adjustAngle
import numpy as np

class Robot(Element):
  """Classe filha que implementa um robô no campo."""

  def __init__(self, controlSystem=UFC("defaultRobot")):
    super().__init__()
    
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

    self.vref = 0

    self.gammavels = (0,0)

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
    return adjustAngle(self.inst_th + (np.pi if self.dir == -1 else 0))

  @th.setter
  def th(self, th):
    """Atualiza o ângulo do objeto diretamente (sem afetar o ângulo anterior)."""
    self.inst_th = th - (np.pi if self.dir == -1 else 0)

  @property
  def raw_th(self):
    """Retorna o theta da visão"""
    return self.inst_th

  @raw_th.setter
  def raw_th(self, th):
    """Atualiza direto a variável que contém o theta da visão"""
    self.inst_th = th

  def update(self, x=0, y=0, th=0, rawUpdate=True):
    """Atualiza a posição do objeto, atualizando também o valor das posições anteriores."""
    if rawUpdate == True:
      super().update(x, y, th)
    else:
      super().update(x, y, th - (np.pi if self.dir == -1 else 0))

    return self