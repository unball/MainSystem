from gi.repository import Gdk
from view.tools.cv2Renderer import cv2Renderer
from view.tools.drawing import Drawing
from controller.tools.pixel2metric import meters2pixel,pixel2meters,meters2pixelSize
import numpy as np
import cv2
import itertools

class HighLevelRenderer(cv2Renderer):
  def __init__(self, world, robotsGetter=None, on_click=None, on_scroll=None):
    super().__init__(worker=self.renderer)
    
    self.__world = world
    
    self.__robotsGetter = robotsGetter
    
    self.__movingRobot = None
    self.__mousePosition = (0,0)
    self.__on_click = on_click
    self.__on_scroll = on_scroll
    self.arrow_size = 15
    self.showField = False
    self.positions = []
    
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
      self.__movingRobot.update(position[0], position[1], self.__movingRobot.th)
  
  def frameClick(self, widget, event):
    """Executado quando um clique é feito na área de renderização, se um robô estiver perto da área de clique, marca o robô como selecionado"""
    nearRobot = self.findNearRobot()
    if nearRobot is not None:
        self.__movingRobot = nearRobot
    else:
      width, height = self.getShape()
      if self.__on_click is not None: self.__on_click(pixel2meters(self.__world, (int(event.x), int(event.y)), (height, width)))
  
  def frameScroll(self, widget, event):
    """Executado quando há evento de scroll, atualiza o ângulo do robô mais próximo"""
    nearRobot = self.findNearRobot()
    if nearRobot is not None:
        nearRobot.th = nearRobot.th+event.delta_y*0.1
    if self.__on_scroll is not None:
      width, height = self.getShape()
      mouse = pixel2meters(self.__world, (int(event.x), int(event.y)), (height, width))
      self.__on_scroll(mouse, event.delta_y)
          
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
    w,h = meters2pixelSize(self.__world, (0.075,0.075), frame.shape)
    Drawing.draw_rectangle(frame, position, (w,h), robot.th, color=robotColor)
    
  def draw_field(self, frame, pose, field):
    """Desenha o campo no frame"""
    if field is None: return
    
    if self.showField:
      # Desenha todo o campo
      x = np.arange(0, frame.shape[1], self.arrow_size)
      y = np.arange(0, frame.shape[0], self.arrow_size)
      pix = np.array(list(itertools.product(x,y)))
      P = np.array(pixel2meters(self.__world, pix.T, frame.shape)).T
      fP = field.F(P.T, retnparray=True)

      for i,p in enumerate(fP):
        Drawing.draw_arrow(frame, (pix[i][0],pix[i][1]), p, color=(128,128,128), size=self.arrow_size, thickness=1)

    # for i in range(0, frame.shape[1], self.arrow_size+2):
    #   for j in range(0, frame.shape[0], self.arrow_size+2):
    #     P = pixel2meters(self.__world, (i,j), frame.shape)
    #     Drawing.draw_arrow(frame, (i,j), field.F(P), color=(128,128,128), size=self.arrow_size, thickness=1)
    
    # Desenha o target do campo
    Drawing.draw_arrow(frame, meters2pixel(self.__world, field.Pb, frame.shape), field.Pb[2], color=(0,255,0), size=meters2pixelSize(self.__world, (0.08,0), frame.shape)[0])

    # Desenha a bola
    cv2.circle(frame, meters2pixel(self.__world, self.__world.ball.pos, frame.shape), meters2pixelSize(self.__world, (0.015,0), frame.shape)[0], color=(0,0,255), thickness=-1)

    # Desenha obstáculos pontuais
    for enemy in self.__world.enemyRobots:
      cv2.circle(frame, meters2pixel(self.__world, enemy, frame.shape), meters2pixelSize(self.__world, (0.010,0), frame.shape)[0], color=(0,255,255), thickness=-1)

    # Desenha o campo na posição do robô
    Drawing.draw_arrow(frame, meters2pixel(self.__world, pose, frame.shape), field.F(pose), color=(0,255,0), size=meters2pixelSize(self.__world, (0.08,0), frame.shape)[0])

    # Desenha pontos de interesse do campo
    #for point in field.interestPoints():
    #  Drawing.draw_arrow(frame, meters2pixel(self.__world, point, frame.shape), point[2], color=(128,128,128), size=arrow_size)
    

    #cv2.polylines(frame,[np.array([meters2pixel(self.__world, pos, frame.shape) for pos in self.positions[-500:]])],False,(255,255,255),1)
  
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
      self.draw_field(frame, robot.pose, robot.field)
      self.draw_robot(frame, robot)
      self.positions.append(robot.pos)
    
    return frame
