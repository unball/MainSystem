from controller.world.element import Element
from controller.tools.kalman import KalmanFilter

def shift(data, array):
    return [data] + array[1:]

class Ball(Element):
  """Classe filha de `Element` que implementa a abstração da bola no mundo."""

  def __init__(self):
    super().__init__()
    self.initialPosSet = False
    self.filter_x = KalmanFilter()
    self.filter_y = KalmanFilter()
    self.vx = .0
    self.vx_ant = [.0]*10
    self.vy_ant = [.0]*10

  def update(self, x=0, y=0, th=0):
    if self.initialPosSet:
      self.filter_x.setInitialPos(x)
      self.filter_y.setInitialPos(y)
    super().update(x,y,th)
    
  def calc_velocities(self, dt, alpha=0.5, thalpha=0.8):


    vx = (self.inst_x-self.prev_x) / dt
    vy = (self.inst_y-self.prev_y) / dt

    self.vx = (vx + sum(self.vx_ant)) / 11
    self.vy = (vy + sum(self.vy_ant)) / 11

    self.vx_ant = shift(vx, self.vx_ant)
    self.vy_ant = shift(vy, self.vy_ant)

    ax = (self.inst_x-2*self.prev_x+self.dprev_x) / dt**2
    ay = (self.inst_y-2*self.prev_y+self.dprev_y) / dt**2

    estimate_x = self.filter_x.estimate(self.inst_x)
    estimate_y = self.filter_y.estimate(self.inst_y)

    self.inst_vx = estimate_x[1,0]
    self.inst_vy = estimate_y[1,0]

    self.inst_ax = estimate_x[2,0]
    self.inst_ay = estimate_y[2,0]

    self.inst_vx = vx
    self.inst_vy = self.vy