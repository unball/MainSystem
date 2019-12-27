from gi.repository import Gtk, GLib
from pkg_resources import resource_filename
from view.tools.frameSelector import FrameRenderer
from view.tools.drawing import Drawing
from controller.tools.pixel2metric import  meters2pixel,meters2pixelSize
import cv2
import numpy as np
import time

class VisaoAltoNivel(FrameRenderer):
  """Essa classe implementa o FrameRenderer que permite ver os dados produzidos pela visão"""
  
  def __init__(self, notebook, controller, visionSystem, world):
    self.__visionSystem = visionSystem
    self.__controller = controller
    self.__world = world
    self.__n_robots = 0
    self.__uiElements = []
    super().__init__(notebook, "Visão em alto nível")
    
  def ui(self):
    """Conteúdo a ser inserido na interface da configuração visão em alto nível"""
    builder = Gtk.Builder.new_from_file(resource_filename(__name__, "visaoAltoNivel.ui"))
    self.__timeFlow = builder.get_object("time_flow")
    self.__timeAdversarioFlow = builder.get_object("time_adversario_flow")
    self.__bolaEstado = builder.get_object("bola_estado")
    self.__bolaPosicao = builder.get_object("bola_posicao")
    return builder.get_object("main")
    
  def getFrame(self):
    """Retorna um frame que desenha retângulos ao redor dos robôs com seus identificadores e círculo ao redor da bola"""
    frame = self.__visionSystem.cameraHandler.getFrame()
    if frame is None: return None
    
    img_warpped = self.__visionSystem.warp(frame)
    img_hsv = self.__visionSystem.converterHSV(img_warpped)
    fgMask = self.__visionSystem.obterMascaraElementos(img_hsv)
    img_filtered = cv2.bitwise_and(img_warpped, img_warpped, mask=fgMask)
    t0 = time.time()
    message = self.__visionSystem.process(frame)
    print((time.time()-t0)*1000)
    GLib.idle_add(self.updateRobotsInfo, message)
    
    Drawing.draw_field(self.__world, img_filtered)
    
    for i in range(message.n_total_robots):
      if(message.found[i]):
        x,y = meters2pixel(self.__world, (message.x[i], message.y[i]), img_warpped.shape)
        w,h = meters2pixelSize(self.__world, (0.08,0.08), img_warpped.shape)
        color = (0,0,255) if i >= message.n_total_robots/2 else (0,255,0)
        Drawing.draw_rectangle(img_filtered, (x,y), (w,h), message.th[i], color=color)
        cv2.putText(img_filtered, str(i), (x-10, y+10), cv2.FONT_HERSHEY_TRIPLEX, 1, (0,0,0))
    
    if message.ball_found:
      p = meters2pixel(self.__world, (message.ball_x, message.ball_y), img_warpped.shape)
      cv2.circle(img_filtered, p, 5, (255,0,0), 2)
      
    return img_filtered
  
  def updateRobotsInfo(self, message):
    """Atualiza os elementos de interface gráfica com os novos valores obtidos pela visão"""
  
    if self.__n_robots != message.n_total_robots:
      for e in self.__uiElements:
        self.__timeFlow.remove(e["fbc"])
      self.__n_robots = message.n_total_robots
      self.__uiElements = [None for x in range(0,self.__n_robots)]
      
    for i in range(self.__n_robots):
      if self.__uiElements[i] is not None:
        self.__uiElements[i]["idLabel"].set_text("{0}".format(i))
        self.__uiElements[i]["posicaoLabel"].set_text("Posição: x: {:.2f} m, y: {:.2f} m".format(message.x[i], message.y[i]))
        self.__uiElements[i]["anguloLabel"].set_text("Ângulo {:.1f}º".format(message.th[i]*180/np.pi))
        self.__uiElements[i]["estadoLabel"].set_text("Estado: " + "Identificado" if message.found[i] else "Não-Identificado")
        if message.found[i]:
          self.__uiElements[i]["fbc"].set_opacity(1)
        else:
          self.__uiElements[i]["fbc"].set_opacity(0.5)
      else:
        flowBoxChild = Gtk.FlowBoxChild()
        Gtk.StyleContext.add_class(flowBoxChild.get_style_context(), "roboRow")
        
        builder = Gtk.Builder.new_from_file(resource_filename(__name__, "robo.ui"))
        idLabel = builder.get_object("idLabel")
        posicaoLabel = builder.get_object("posicaoLabel")
        estadoLabel = builder.get_object("estadoLabel")
        anguloLabel = builder.get_object("anguloLabel")
        
        if i < self.__n_robots/2:
          Gtk.StyleContext.add_class(flowBoxChild.get_style_context(), "roboAliado")
          builder.get_object("tipoRoboLabel").set_text("Tipo: Aliado")
          self.__timeFlow.add(flowBoxChild)
        else:
          Gtk.StyleContext.add_class(flowBoxChild.get_style_context(), "roboInimigo")
          builder.get_object("tipoRoboLabel").set_text("Tipo: Inimigo")
          self.__timeAdversarioFlow.add(flowBoxChild)
        
        flowBoxChild.add(builder.get_object("main"))
        self.__uiElements[i] = {"idLabel": idLabel, "posicaoLabel": posicaoLabel, "anguloLabel": anguloLabel, "estadoLabel": estadoLabel, "fbc": flowBoxChild}
        
    self.__timeFlow.show_all()
    self.__timeAdversarioFlow.show_all()
    
    if message.ball_found:
      self.__bolaEstado.set_text("Estado: Identificada")
      self.__bolaPosicao.set_text("Posição: x: {:.2f} m, y: {:.2f} m".format(message.ball_x, message.ball_y))
    else:
      self.__bolaEstado.set_text("Estado: Não-Identificada")
      
