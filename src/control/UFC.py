from tools import norm, ang, angError, sat, speeds2motors, fixAngle, filt
from tools.interval import Interval
import numpy as np
import math
import time 

class UFC():
  """Controle unificado para o Univector Field, utiliza o ângulo definido pelo campo como referência \\(\\theta_d\\)."""
  def __init__(self):
    self.g = 9.8
    self.kw = 5.5
    self.kp = 10
    self.mu = 0.7
    self.amax = self.mu * self.g
    self.vmax = 2
    self.L = 0.075

    self.lastth = 0
    self.interval = Interval(filter=True, initial_dt=0.016)

  def actuate(self, robot):
    if robot.field is None: return 0,0
    # Ângulo de referência
    th = robot.field.F(robot.pose)
    # Erro angular
    eth = angError(th, robot.th)
    # Tempo desde a última atuação
    dt = self.interval.getInterval()
    # Derivada da referência
    dth = sat(angError(th, self.lastth) / dt, 15)
    # Computa phi
    phi = robot.field.phi(robot.pose)
    # Computa gamma
    gamma = robot.field.gamma(dth, robot.velmod, phi)
    #print("eth: {:.4f}\tphi: {:.4f}\tth: {:.4f}\trth: {:.4f}\t".format(eth*180/np.pi, phi, th*180/np.pi, robot.th*180/np.pi))
    
    # Computa omega
    omega = self.kw * np.sign(eth) * np.sqrt(np.abs(eth)) + gamma

    # Velocidade limite de deslizamento
    if phi != 0: v1 = (-np.abs(omega) + np.sqrt(omega**2 + 4 * np.abs(phi) * self.amax)) / (2*np.abs(phi))
    if phi == 0: v1 = self.amax / np.abs(omega)

    # Velocidade limite das rodas
    v2 = (2*self.vmax - self.L * np.abs(omega)) / (2 + self.L * np.abs(phi))

    # Velocidade limite de aproximação
    v3 = self.kp * norm(robot.pos, robot.field.Pb) ** 2 + robot.vref

    # Velocidade linear é menor de todas
    v  = max(min(v1, v2, v3), 0)

    # Lei de controle da velocidade angular
    w = v * phi + omega
    
    # Atualiza a última referência
    self.lastth = th
    robot.lastControlLinVel = v
    
    if robot.spin == 0: return speeds2motors(v * robot.direction, w)
    else: return speeds2motors(0, 60 * robot.spin)