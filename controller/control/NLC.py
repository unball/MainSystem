from controller.control import SpeedPair
import numpy as np

class RobotVariable():
  def __init__(self, ka, kg, kp):
    # Valores atuais
    self.x_i = 0
    self.y_i = 0
    self.th_i = 0
    
    # Valores de referência
    self.x_r = 0
    self.y_r = 0
    self.th_r = 0
    
    # Erros
    self.x_e = 0
    self.y_e = 0
    self.th_e = 0
    
    # Constantes de controle
    self.ka = ka
    self.kg = kg
    self.kp = kp

class NLC():
    def __init__(self, world):
      
      self.world = world
      
      # Mantém o número de robôs
      self.number_of_robots = world.n_robots
      
      # Saída do controle
      self.output_vel = [SpeedPair() for i in range(self.number_of_robots)]
      
      # Constantes do controle
      ka = np.array([7, 3.5, 3*0.8, 4, 4])
      kg = np.array([800, 3.5, 3*0.8, 4, 4])
      kp = np.array([10, 4.5, 3.75*1.7, 3.75, 3.75])
      
      # Variáveis de controle para cada robô
      self.robots = [RobotVariable(ka[i], kg[i], kp[i]) for i in range(self.number_of_robots)]
      
    def setConstantes(self, ka, kg, kp):
      self.robots[0].ka = ka
      self.robots[0].kg = kg
      self.robots[0].kp = kp
    
    def updateIntVariables(self, references):
      for i,robot in enumerate(self.robots):
        robot.th_i = self.world.robots[i].th
        robot.x_i = self.world.robots[i].x
        robot.y_i = self.world.robots[i].y
        
        robot.th_r = references[i][2]
        robot.x_r = references[i][0]
        robot.y_r = references[i][1]
    
    def sat(self, x, amp):
        return max(min(x, amp), -amp)
    
    def updateError(self):
      for i,robot in enumerate(self.robots):
        robot.x_e = robot.x_r - robot.x_i
        robot.y_e = robot.y_r - robot.y_i
        robot.th_e = robot.th_r - robot.th_i
        
        robot.th_e = self.fixAngle(robot.th_e)
          
    def fixAngle(self, angle):
      if abs(angle) > np.pi/2:
        return (angle + np.pi/2) % (np.pi) - np.pi/2
      else:
        return angle
    
    def santitize(self, references):
      santitized = [(0,0,0) for i in range(len(self.robots))]
      for i in range(min(len(references), len(self.robots))):
        santitized[i] = references[i]
        
      return santitized

    def actuate(self, references, spinList):
        """Control system actuator itself."""
        self.updateIntVariables(self.santitize(references))
        self.updateError()
        self.controlLaw()
        
        return self.output_vel
    
    def controlLaw(self):
      for i,robot in enumerate(self.robots):
        p = np.sqrt(robot.x_e**2+robot.y_e**2)
        gamma = np.arctan2(robot.y_e, robot.x_e)
        
        self.output_vel[i].w = robot.ka * robot.th_e + robot.kg * self.fixAngle(gamma-robot.th_i) * p**2
        
        self.output_vel[i].w = self.sat(self.output_vel[i].w, 4*np.pi)
        
        # !TODO: Portar elements e robot do main_system antigo
#        if self.world.robots[i].velmod < 0.01:
#          sigma = robot.th_i
#        else:
#          sigma = self.world.robots[i].velang
        sigma = robot.th_i
        
        self.output_vel[i].v = robot.kp * p * np.cos(gamma-sigma) / np.cos(sigma-robot.th_i)
        
        
