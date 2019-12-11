from gi.repository import Gdk
from view.cv2renderer import cv2Renderer
from view.drawing import Drawing
from controller.vision.pixel2metric import meters2pixel,pixel2meters,meters2pixelSize
import numpy as np
import cv2

class HighLevelRenderer(cv2Renderer):
  def __init__(self, world, robotsGetter=None):
    super().__init__(worker=self.renderer)
    
    self.__world = world
    
    self.__robotsGetter = robotsGetter
    
    self.__movingRobot = None
    self.__mousePosition = (0,0)
    
    # Adiciona eventos de mouse e trackpad
    eventBox = self.getEventBox()
    eventBox.connect("motion-notify-event", self.frameMouseOver)
    eventBox.connect("button-press-event", self.frameClick)
    eventBox.connect("button-release-event", self.frameRelease)
    eventBox.add_events(Gdk.EventMask.SMOOTH_SCROLL_MASK)
    eventBox.connect("scroll-event", self.frameScroll)
    
  @property
  def robots(self):
    if self.__robotsGetter is None: return []
    return self.__robotsGetter()
  
  def cursorDistance(self, position: tuple):
    """Calcula a distância da posição atual do mouse a `position`"""
    return abs(self.__mousePosition[0]-position[0])+abs(self.__mousePosition[1]-position[1])
    
  def findNearRobot(self):
    """Encontra um robô na lista de robos que esteja próximo do mouse"""
    width, height = self.getShape()
    nearRobots = [r for r in self.robots if self.cursorDistance(meters2pixel(self.__world, r.pose, (height, width))) < 20]
    if(len(nearRobots) != 0):
      return nearRobots[0]
    else: return None
      
  def frameMouseOver(self, widget, event):
    """Executado quando um mouse passa por cima da área de renderização, se um robô tiver sido selecionado, atualiza sua posição"""
    width, height = self.getShape()
    self.__mousePosition = (int(event.x), int(event.y))
    if self.__movingRobot is not None:
      position = pixel2meters(self.__world, (int(event.x), int(event.y)), (height, width))
      self.__movingRobot.pose = (position[0], position[1], self.__movingRobot.th)
  
  def frameClick(self, widget, event):
    """Executado quando um clique é feito na área de renderização, se um robô estiver perto da área de clique, marca o robô como selecionado"""
    nearRobot = self.findNearRobot()
    if nearRobot is not None:
        self.__movingRobot = nearRobot
  
  def frameScroll(self, widget, event):
    """Executado quando há evento de scroll, atualiza o ângulo do robô mais próximo"""
    nearRobot = self.findNearRobot()
    if nearRobot is not None:
        nearRobot.th = nearRobot.th+event.delta_y*0.1
          
  def frameRelease(self, widget, event):
    """Executado quando um clique é solto, faz o robô selecionado ficar `None`"""
    self.__movingRobot = None
    
  def draw_robot(self, frame, robot):
    """Desenha um robô e seu target no frame"""
    # Obtém posição em pixels do robô
    position = meters2pixel(self.__world, robot.pose, frame.shape)
    
    # Desenha de outra cor o robô próximo do mouse
    robotColor = (255,0,0) if self.cursorDistance(position) < 20 else (0,255,0)
    
    # Desenha o retângulo do robô
    w,h = meters2pixelSize(self.__world, (0.08,0.08), frame.shape)
    Drawing.draw_rectangle(frame, position, (w,h), robot.th, color=robotColor)
    
    # Desenha o target instantâneo do robô
    if robot.trajectory is not None:
      target = robot.trajectory.target(robot.pose, robot.step)
      Drawing.draw_arrow(frame, meters2pixel(self.__world, target, frame.shape), target[2], color=(0,255,0))
    
  def draw_trajectory(self, frame, trajectory):
    """Desenha uma trajetória no frame"""
    if trajectory is None: return
    
    cv2.polylines(frame,[np.array([meters2pixel(self.__world, trajectory.P(t), frame.shape) for t in np.linspace(0,1)])],False,(255,255,255),1)
  
  def renderer(self):
    """Desenha um frame para o renderizador"""
    
    # Obtém o tamanho adequado a renderizar
    width, height = self.getShape()
    
    # Cria um frame do tamanho adequado com tudo preto
    frame = np.zeros((height,width,3), np.uint8)
    
    # Desenha os lados de campo
    Drawing.draw_field(self.__world, frame)
    
    # Desenha os robôs e suas trajetórias
    for i,robot in enumerate(self.robots):
      self.draw_robot(frame, robot)
      self.draw_trajectory(frame, robot.trajectory)
    
    return frame
