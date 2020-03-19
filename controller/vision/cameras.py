import cv2
import time
from pkg_resources import resource_filename
from os import listdir
from threading import Lock

from model.paramsPattern import ParamsPattern

class CameraHandler(ParamsPattern):
  """Classe que gerencia as câmeras do sistema permitindo rápida troca e retorno simples de frames"""
  def __init__(self):
    ParamsPattern.__init__(self, "cameraParams", {
      "current_camera": -1,
      "frame_scale": 1,
      "frame_width": 640,
      "frame_height": 480,
      "frame_fps": 30,
      "frame_format": "YUYV"
    })

    self.__cameras = []
    """Conjunto de câmeras disponíveis"""
    
    self.__defaultFrame = cv2.imread(resource_filename(__name__, "defaultFrame.png"))
    """Armazena o frame padrão"""
    
    self.__cap = None
    """Armazena o `VideoCapture` atual"""

    self.__lock = Lock()
    
  def setScale(self, scale: float):
    """Altera o valor da escala usada no frame"""
    self.setParam("frame_scale", scale)

  def setCameraParams(self, values):
    self.__lock.acquire()
    if self.__cap is None: return

    self.setParam("frame_format", values["frame_format"])
    self.setParam("frame_width", values["frame_width"])
    self.setParam("frame_height", values["frame_height"])
    self.setParam("frame_fps", values["frame_fps"])

    self.setCamera(self.getParam("current_camera"), lock=False)

    fourcc = int(self.__cap.get(cv2.CAP_PROP_FOURCC))
    fourccstr = fourcc.to_bytes(length=4, byteorder='little').decode()

    self.setParam("frame_format", fourccstr)
    self.setParam("frame_width", int(self.__cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    self.setParam("frame_height", int(self.__cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    self.setParam("frame_fps", int(self.__cap.get(cv2.CAP_PROP_FPS)))

    self.__lock.release()
    
  def getScale(self):
    """Obtém o valor da escala usada no frame"""
    return self.getParam("frame_scale")
    
  def scaleFrame(frame, scale: float):
    """Muda o tamanho, mantendo a proporção, de um frame de acordo com uma escala."""
    scale = max(scale, 0.01)
    return cv2.resize(frame, (round(frame.shape[1]*scale), round(frame.shape[0]*scale)))
    
  def getFrame(self):
    """Retorna um frame no formato numpy (height, width, depth) com base na câmera atualmente selecionada."""
    try:
      if self.getParam("current_camera") == -1:
        time.sleep(0.001)
        if self.__defaultFrame is None: return None
        return CameraHandler.scaleFrame(self.__defaultFrame, self.getParam("frame_scale"))

      self.__lock.acquire()
      
      if self.__cap is None:
        self.setCamera(self.getParam("current_camera"), lock=False)
      
      if self.__cap is not None and self.__cap.isOpened():
        ret, frame = self.__cap.read()
        if frame is None: 
          time.sleep(0.001)
          self.__lock.release()
          return None
        self.__lock.release()
        return CameraHandler.scaleFrame(frame, self.getParam("frame_scale"))

      self.__lock.release()
    except Exception as e: 
      print(e)
      print("Falha no módulo CameraHandler getFrame")
      time.sleep(0.001)
      return None
      
    return None
    
  def setCamera(self, index, lock=True):
    """Seleciona a câmera de índice `index`"""
    if lock: self.__lock.acquire()
    # Libera a câmera em uso
    if self.__cap is not None: self.__cap.release()
    
    # Aloca a nova câmera
    if index != -1: 
      #cap = cv2.VideoCapture("../visao/UnBall&RobotBulls1.webm")
      cap = cv2.VideoCapture(index)
      cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*list(self.getParam("frame_format"))))
      cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.getParam("frame_width"))
      cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.getParam("frame_width"))
      cap.set(cv2.CAP_PROP_FPS, self.getParam("frame_fps"))
      
      self.__cap = cap

    if lock: self.__lock.release()
    
    self.setParam("current_camera", index)
    
  def getCamera(self):
    """Retorna o índice de câmera selecionado"""
    return self.getParam("current_camera")
    
  def updateCameras(self):
    """Atualiza a lista de câmeras detectadas"""
    self.__cameras = [c for c in listdir("/sys/class/video4linux/")]
    
  def getCameras(self):
    """Retorna as câmeras detectadas"""
    return self.__cameras
    
    
