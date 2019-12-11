from gi.repository import Gtk
from pkg_resources import resource_filename
from view.stackSelector import StackSelector
from view.strategy.highLevelRenderer import HighLevelRenderer
from controller.states.debugHLC import DebugHLC
import numpy as np

class HLCView(StackSelector):
  """Classe que gerencia a view de depurador do controle de alto nível"""
  
  def __init__(self, controller, world, stack):
    self.__controller = controller
    self.__world = world
    super().__init__(stack, "configHLC", "HLC")
  
  def ui(self):
    # Carrega os elementos estáticos
    builder = Gtk.Builder.new_from_file(resource_filename(__name__, "HLC.ui"))
    
    # Elementos internos
    mainBox = builder.get_object("HLCBox")
    playPause = builder.get_object("HLCPlayPause")
    
    self.__renderer = HighLevelRenderer(self.__world, robotsGetter=self.robotsGetter)
    """Instancia o renderizador, ele é do tipo GtkFrame"""
    
    # Adiciona o renderizador ao GtkBox
    mainBox.pack_start(self.__renderer, True, True, 0)
    mainBox.reorder_child(self.__renderer, 0)
    
    playPause.connect("toggled", self.playPause)
    
    return mainBox
    
  def robotsGetter(self):
    return self.__world.getRobots()[:1]
    
  def playPause(self, widget):
    self.__controller.addEvent(self.__world.setRunning, widget.get_active())
    
  def on_select(self, widget):
    self.__controller.addEvent(self.__controller.setState, DebugHLC(self.__controller))
    self.__renderer.start()
    
  def on_deselect(self, widget):
    self.__controller.addEvent(self.__controller.unsetState)
    self.__renderer.stop()
