from controller.world.element import Element

class Ball(Element):
  """Classe filha de `Element` que implementa a abstração da bola no mundo."""

  def __init__(self):
    super().__init__()
    
  def calc_velocities(self, dt, alpha=0.5, thalpha=0.8):
    super().calc_velocities(dt, alpha=0.5)
