class VisionMessage():
  """Classe que descreve uma mensagem de alteração da visão. Essa mensagem contém novos valores para pose de robôs e bola, além de uma lista que indica se o robô foi identificado ou não."""
  
  def __init__(self, n_robots):
    """Constroi a mensagem da visão com o pior cenário possível: nada identificado e todos os elementos na posição central"""
    
    self.n_total_robots = 2*n_robots
    """Número de robôs aliados e adversários"""
    
    self.x = [0]*self.n_total_robots
    """Lista com coordenada x dos robôs (aliados e inimigos)"""
    
    self.y = [0]*self.n_total_robots
    """Lista com coordenada y dos robôs (aliados e inimigos)"""
    
    self.th = [0]*self.n_total_robots
    """Lista com ângulo dos robôs (aliados e inimigos)"""
    
    self.debug_internalContours = [None]*self.nRobots
    """Lista os contornos internos de aliados"""
    
    self.ball_x = 0
    """Coordenada x da bola"""
    
    self.ball_y = 0
    """Coordenada y da bola"""
    
    self.ball_found = False
    """Indica se a bola foi encontrada"""
    
    self.found = [False]*self.n_total_robots
    """Lista de booleano que indica se o robô (aliado ou inimigo) foi ou não identificado"""
    
    self.advIndex = n_robots
    """Índice onde estará o próximo robô adversário adicionado"""
    
  @property
  def nRobots(self):
    """Retorna o número de robôs aliados"""
    return self.n_total_robots//2
  
  def setRobot(self, index, pose, internalContours=None):
    """Diz que o robô de índice `index` foi identificado e atualiza sua pose"""
    if(index < self.n_total_robots):
      self.x[index] = pose[0]
      self.y[index] = pose[1]
      self.th[index] = pose[2]
      self.found[index] = True
    
    if index < self.nRobots:
      self.debug_internalContours[index] = internalContours
    
  def setEnemyRobot(self, pose):
    """Coloca um robô inimigo como identificado e atualiza sua pose"""
    self.setRobot(self.advIndex, pose)
    self.advIndex = self.advIndex + 1
  
  def setBall(self, pos):
    """Coloca a bola como identificada e atualiza sua posição"""
    
    if pos is None: return
    self.ball_x, self.ball_y = pos[0]
    self.ball_found = True
