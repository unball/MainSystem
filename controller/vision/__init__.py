from abc import ABC, abstractmethod
import time

from controller.vision.cameras import CameraHandler
from controller.vision.visionMessage import VisionMessage

class Vision(ABC):
  """Classe que define as interfaces que qualquer sistema de visão deve ter no sistema."""
  
  def __init__(self, world):
    super().__init__()
    
    self.cameraHandler = CameraHandler()
    """Instancia módulo `MainSystem.controller.vision.cameras.CameraHandler` que gerencia as câmeras e retorna os frames"""
    
    self._world = world
    """Mantém referência ao mundo"""
  
  @abstractmethod
  def process(self, frame):
    """Método abstrato que recebe um frame do tipo numpy array no formato (height, width, depth). Retorna uma mensagem de alteração do tipo `MainSystem.controller.vision.visionMessage.VisionMessage`"""
    pass
    
  def giveUpAndWait(self):
    """Método que impõe um atraso de 30ms por falta de frame."""
    time.sleep(0.03)
    return False
  
  def update(self):
    """Obtém um frame da câmera, chama o `Vision.process` e atualiza o mundo (`World`) com base na mensagem retornada. Retorna `False` se nada foi feito e `True` se atualizou o mundo."""
    
    frame = self.cameraHandler.getFrame()
    if frame is None: return self.giveUpAndWait()
    
    self._world.update(self.process(frame))
    
    return True
