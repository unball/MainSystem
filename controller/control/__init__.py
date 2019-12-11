class SpeedPair():
  """Classe que mantém velocidade angular e linear de atuação do controle"""
  def __init__(self):
    self.v = 0
    self.w = 0
  
  def __repr__(self):
    """Faz o pretty-print das velocidades"""
    return "{" + "v: {0}, w: {1}".format(self.v, self.w) + "}"
