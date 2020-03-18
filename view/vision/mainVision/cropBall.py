from gi.repository import Gtk
from pkg_resources import resource_filename
from view.tools.frameSelector import FrameRenderer
import cv2
import numpy as np

class CropBall(FrameRenderer):
  """Essa classe implementa o FrameRenderer que permite configurar os filtros morfológicos"""
  
  def __init__(self, notebook, controller, visionSystem):
    self.__visionSystem = visionSystem
    self.__controller = controller
    super().__init__(notebook, "CropBall")
    
  def ui(self):
    """Conteúdo a ser inserido na interface da configuração de parâmetros da visão"""
    builder = Gtk.Builder.new_from_file(resource_filename(__name__, "morfologia.ui"))
    
    return builder.get_object("main")
  
  def getFrame(self):
    """Retorna a máscara do que é elemento no campo após os filtros morfológicos"""
    frame = self.__visionSystem.cameraHandler.getFrame()
    if frame is None: return None

    return cv2.cvtColor(self.__visionSystem.process(frame, checkpoint="crop")[1], cv2.COLOR_HSV2BGR)