from gi.repository import Gtk
from pkg_resources import resource_filename
from view.tools.frameSelector import FrameRenderer
from controller.tools import norm
from controller.tools.pixel2metric import normToAbs
from model.vision.cortarCampoInternoModel import CortarCampoInternoModel
import cv2
import numpy as np

class CortarCampoInterno(FrameRenderer):
  """FrameRenderer que implementa a configuração de corte de campo interno, cuja função é definir o polígono que envolverá a região permitida dos robôs se moverem."""

  def __init__(self, notebook, controller, visionSystem, eventBox):
    self.__visionSystem = visionSystem
    self.__controller = controller
    self.__eventBox = eventBox

    self.__model = CortarCampoInternoModel()
    """Mantém as variáveis permanentes do módulo view de cortar campo interno"""

    self.__current_mouse_position = (0,0)
    """Mantém a posição do mouse em cima do frame"""

    self.__edgeType = "repulsive"

    self.__edgeColors = {"repulsive": (0,0,255), "goalRepulsive": (0,100,255), "goalAttractive": (0,255,0)}

    super().__init__(notebook, "Cortar campo interno")

  def ui(self):
    """Conteúdo a ser inserido na interface da configuração de cortar campo"""
    builder = Gtk.Builder.new_from_file(resource_filename(__name__, "cortarCampoInterno.ui"))
    self.__eventBox.connect("button-press-event", self.update_points)
    self.__eventBox.connect("motion-notify-event", self.mouse_over)

    builder.get_object("OpArestaRepulsiva").connect("toggled", self.setEdgeType, "repulsive")
    builder.get_object("OpArestaRepulsivaGol").connect("toggled", self.setEdgeType, "goalRepulsive")
    builder.get_object("OpArestaAtrativaGol").connect("toggled", self.setEdgeType, "goalAttractive")
    builder.get_object("ButReset").connect("clicked", self.resetPoints)

    return builder.get_object("main")

  def connectSpecialSignals(self):
    """Desbloqueia os sinais de clique e mouse sobreposto"""
    self.__eventBox.handler_unblock_by_func(self.update_points)
    self.__eventBox.handler_unblock_by_func(self.mouse_over)

  def disconnectSpecialSignals(self):
    """Bloqueia os sinais de clique e mouse sobreposto"""
    self.__eventBox.handler_block_by_func(self.update_points)
    self.__eventBox.handler_block_by_func(self.mouse_over)

  def setEdgeType(self, widget, edgeType):
    self.__edgeType = edgeType

  def resetPoints(self, widget):
    self.__model.points = []
    self.__model.done = False

  def getRelPoint(self, widget, event):
    """Com base no evento e no widget de eventBox, calcula a posição relativa do mouse no frame"""
    x = event.x/widget.get_allocated_width()
    y = event.y/widget.get_allocated_height()

    return (x,y)

  def nearPoint(self, point):
    for p in self.__model.points:
      if norm(point, p[0]) < 0.02:
        return p
    return None

  def pointReturnedToStart(self, point):
    nearPoint = self.nearPoint(point)
    return nearPoint is not None and nearPoint == self.__model.points[0]

  def update_points(self, widget, event):
    """Atualiza a lista de pontos do polígono"""

    # Se o polígono foi fechado, não faz nada
    if self.__model.done is True: return

    point = self.getRelPoint(widget, event)

    if self.pointReturnedToStart(point):
      self.__model.done = True
    else:
      self.__model.points.append((point, self.__edgeType))

  def mouse_over(self, widget, event):
    """Evento quando o mouse sobrepõe o frame, este método atualiza a posição relativa do mouse"""
    self.__current_mouse_position = self.getRelPoint(widget, event)

  def getFrame(self):
    """Produz o frame a ser renderizado pelo modo de configuração do corte do campo interno"""
    frame = self.__visionSystem.cameraHandler.getFrame()
    if frame is None: return None

    frame = self.__visionSystem.warp(frame)

    # Defineo que será renderizado
    if self.__model.done is True:
      points = self.__model.points + [self.__model.points[0]]
    else:
      points = self.__model.points + [(self.__current_mouse_position, self.__edgeType)]

    # Renderiza as arestas do polígono
    for i in range(len(points)-1):
      p0 = normToAbs(points[i][0], frame.shape)
      p1 = normToAbs(points[i+1][0], frame.shape)

      color = self.__edgeColors[points[i+1][1]]

      cv2.line(frame, p0, p1, color, thickness=2)

    # Renderiza um círculo ao redor do ponto de início
    if self.__model.done is False and self.pointReturnedToStart(self.__current_mouse_position):
      center = normToAbs(self.__model.points[0][0], frame.shape)
      cv2.circle(frame, center, 10, (255,255,255), -1)

    # Preenche o polígono
    if self.__model.done is True:
      mask = np.zeros((*frame.shape[:2],1), np.uint8)
      pts = np.array([normToAbs(x[0], frame.shape) for x in self.__model.points])
      cv2.fillConvexPoly(mask, pts, 255)
      frame = cv2.bitwise_and(frame, frame, mask=mask)

    return frame
