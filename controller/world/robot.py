from controller.world.element import Element

class Robot(Element):
  """Classe filha que implementa um robô no campo."""

  def __init__(self):
    super().__init__()
    
    self.step = 0.03
    """Distância ao próximo target da trajetória"""
    
    self.__trajectory = None
    """Trajetória definida para este robô"""


  @property
  def trajectory(self):
    """Retorna a trajetória definida para este robô"""
    return self.__trajectory

  @trajectory.setter
  def trajectory(self, trajectory):
    """Atualiza a trajetória definida para este robô"""
    self.__trajectory = trajectory
