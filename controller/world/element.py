import numpy as np

class Element(object):
  """Classe mãe que implementa um elemento de jogo como bola ou robô"""

  def __init__(self):
    self.inst_x = 0
    """Posição x atual"""
    
    self.inst_y = 0
    """Posição y atual"""
    
    self.inst_th = 0
    """Ângulo atual"""

    self.prev_x = 0
    """Posição x anterior"""
    
    self.prev_y = 0
    """Posição y anterior"""
    
    self.prev_th = 0
    """Ângulo anterior"""

    self.inst_vx = 0
    """Estimativa da velocidade na direção x"""
    
    self.inst_vy = 0
    """Estimativa da velocidade na direção y"""
    
    self.inst_w = 0
    """Estimativa da velocidade angular"""
    
    self.poseDefined = False
    """Flag que indica se a pose já foi definida alguma vez via `update`"""
  
  def __repr__(self):
    """Representação formal do objeto. Retorna um pretty-printer dos atributos"""
    x = 'x: ' + str(self.inst_x) + '\n'
    y = 'y: ' + str(self.inst_y) + '\n'
    th = 'th: ' + str(self.inst_th) + '\n'
    vx = 'vx: ' + str(self.inst_vx) + '\n'
    vy = 'vy: ' + str(self.inst_vy) + '\n'
    w = 'w: ' + str(self.inst_w)
    info = x + y + th + vx + vy + w
    return info

  def update(self, x=0, y=0, th=0):
    """Atualiza a posição do objeto, atualizando também o valor das posições anteriores."""
    self.prev_x = self.inst_x
    self.prev_y = self.inst_y
    self.prev_th = self.inst_th
    self.inst_x = x
    self.inst_y = y
    self.inst_th = th
    self.poseDefined = True
    return self

  @property
  def pos(self):
    """Retorna a posição \\([x,y]\\) do objeto como uma lista."""
    return [self.inst_x, self.inst_y]

  @property
  def pose(self):
    """Retorna a pose \\([x,y,\\theta]\\) do objeto um numpy array."""
    return np.array([self.inst_x, self.inst_y, self.inst_th])

  @pos.setter
  def pos(self, x, y):
    """Atualiza a posição do objeto diretamente (sem afetar os valores anteriores)."""
    self.inst_x = x
    self.inst_y = y

  @property
  def th(self):
    """Retorna o ângulo do objeto"""
    return self.inst_th

  @th.setter
  def th(self, th):
    """Atualiza o ângulo do objeto diretamente (sem afetar o ângulo anterior)."""
    self.inst_th = th

  @property
  def vel(self):
    """Retorna a velocidade do objeto no formato de lista: \\([v_x, v_y]\\)"""
    return [self.inst_vx, self.inst_vy]

  @property
  def velmod(self):
    """Retorna o módulo da velocidade do objeto: \\(\\sqrt{v_x^2+v_y^2}\\)"""
    return np.sqrt(self.inst_vx**2+self.inst_vy**2)

  @property
  def velang(self):
    """Retorna o ângulo do vetor velocidade do objeto: \\(\\text{arctan2}(v_y, v_x)\\)"""
    return np.arctan2(self.inst_vy, self.inst_vx)
    
  @vel.setter
  def vel(self, v):
    """Atualiza a velocidade do objeto recebendo uma tupla `v` no formato \\((v_x,v_y)\\)"""
    vx, vy = v
    self.inst_vx = vx
    self.inst_vy = vy

  @property
  def w(self):
    """Retorna a velocidade angular do objeto"""
    return self.inst_w

  @w.setter
  def w(self, w):
    """Atualiza a velocidade angular do objeto"""
    self.inst_w = w

  def calc_velocities(self, dt, alpha=0.5):
    """Estima a velocidade do objeto por meio do pose atual, pose anterior e o intervalo de tempo passado `dt`. A velocidade computada é suavizada por uma média exponencial: \\(v[k] = v_{\\text{estimado}} \\cdot \\alpha + v[k-1] \\cdot (1-\\alpha)\\) onde \\(v_{\\text{estimado}} = \\frac{r[k]-r[k-1]}{dt}\\)"""
    self.inst_vx = ((self.inst_x - self.prev_x)/dt)*alpha + (self.inst_vx)*(1-alpha)
    self.inst_vy = ((self.inst_y - self.prev_y)/dt)*alpha + (self.inst_vy)*(1-alpha)
    self.inst_w = ((self.inst_th - self.prev_th)/dt)*alpha + (self.inst_w)*(1-alpha)

  @property
  def x(self):
    """Retorna a posição \\(x\\) atual do objeto"""
    return self.inst_x

  @property
  def y(self):
    """Retorna a posição \\(y\\) atual do objeto"""
    return self.inst_y

