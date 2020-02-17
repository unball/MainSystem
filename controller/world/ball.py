from controller.world.element import Element
from controller.tools.kalman import KalmanFilter

def shift(data, array):
    return [data] + array[1:]

class Ball(Element):
  """Classe filha de `Element` que implementa a abstração da bola no mundo."""

  def __init__(self, world):
    super().__init__(world)