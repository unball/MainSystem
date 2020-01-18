from controller.world.element import Element
from controller.control.UFC import UFC

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
    """Direção de movimento do robô"""

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
