from gi.repository import Gtk
from pkg_resources import resource_filename
from view.tools.frameSelector import FrameRenderer
import cv2
import numpy as np

class Morfologia(FrameRenderer):
  """Essa classe implementa o FrameRenderer que permite configurar os filtros morfológicos"""
  
  def __init__(self, notebook, controller, visionSystem):
    self.__visionSystem = visionSystem
    self.__controller = controller
    super().__init__(notebook, "Componentes conectados")
    
  def ui(self):
    """Conteúdo a ser inserido na interface da configuração de parâmetros da visão"""
    builder = Gtk.Builder.new_from_file(resource_filename(__name__, "morfologia.ui"))
    
    return builder.get_object("main")
  
  def getFrame(self):
    """Retorna a máscara do que é elemento no campo após os filtros morfológicos"""
    frame = self.__visionSystem.cameraHandler.getFrame()
    if frame is None: return None


    # Requisitos
    img_warpped = self.__visionSystem.warp(frame)
    img_hsv = self.__visionSystem.converterHSV(img_warpped)
    fgMask = self.__visionSystem.obterMascaraElementos(img_hsv)
    teamMask = self.__visionSystem.obterMascaraTime(img_hsv)
    ballMask = self.__visionSystem.obterMascaraBola(img_hsv)
    fgMaskNoBall = cv2.bitwise_and(fgMask, cv2.bitwise_not(ballMask))

    # Componentes conectados
    components = self.__visionSystem.obterComponentesConectados(fgMaskNoBall)

    if len(components) == 0: return None

    blank_ch = np.zeros_like(components[0])
    hue = np.zeros_like(components[0]).astype(np.float64)

    for i,component in enumerate(components):
      hue += component * ((i)/(len(components)+1)*179)
      blank_ch[component != 0] = 255

    hsv = cv2.merge([(hue).astype(np.uint8), blank_ch, blank_ch])
    rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    #rgb[hue==0] = 0
    return rgb