from controller.world.element import Element
from controller.control.NLC import NLC

class Robot(Element):
  """Classe filha que implementa um robô no campo."""

  def __init__(self, controlSystem=NLC("defaultRobot")):
    super().__init__()
    
    self.step = 0.03
    """Distância ao próximo target da trajetória"""
    
    self.__trajectory = None
    """Trajetória definida para este robô"""
    
    self.__controlSystem = controlSystem
    """Sistema de controle para este robô"""

  @property
  def trajectory(self):
    """Retorna a trajetória definida para este robô"""
    return self.__trajectory

  @trajectory.setter
  def trajectory(self, trajectory):
    """Atualiza a trajetória definida para este robô"""
    self.__trajectory = trajectory

  @property
  def controlSystem(self):
    """Retorna o sistema de controle do robô"""
    return self.__controlSystem

  @controlSystem.setter
  def controlSystem(self, controlSystem):
    """Atualiza o sistema de controle do robô"""
    self.__controlSystem = controlSystem
