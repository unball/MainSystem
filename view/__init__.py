import gi
gi.require_version('Gtk', '3.0')

from gi.repository import Gtk, Gdk
from pkg_resources import resource_filename

from view.vision.camerasView import CameraHandlerView
from view.vision.mainVision.mainVisionView import MainVisionView
from view.HLC.HLCView import HLCView
from view.viewThreads import ViewThreads

class View:
  def __init__(self, controller):
    self.__controller = controller
    """Mantém uma referência ao controller"""
    
    self.__threads = ViewThreads()
    """Contém uma lista de threads de view que poderão ser finalizadas quando o programa acabar"""
    
  def on_destroy(self, window):
    """Este método é chamado quando a janela é fechada e faz com que a thread de backend acabe"""
    self.__controller.stop()
    self.__threads.stop()
    Gtk.main_quit()
    
  def registerThread(self, thread):
    """Registra uma thread de view para que possa ser finalizada quando o programa fechar"""
    self.__threads.register(thread)
  
  def run(self):
    """Executa o loop principal do Gtk e instancia os elementos de interface gráfica da janela principal"""
    
    # Adiciona os elementos estáticos da janela principal da interface
    builder = Gtk.Builder.new_from_file("view/main.ui")
    
    # Load CSS
    css_provider = Gtk.CssProvider()
    css_provider.load_from_path(resource_filename(__name__, "style.css"))
    Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), css_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
    
    # Cria a janela
    window = builder.get_object("mainWindow")
    window.show_all()
    
    # Conecta o sinal de saída do programa
    window.connect("destroy", self.on_destroy)
    
    # Botão que gerencia as câmeras
    CameraHandlerView(self.__controller, self.__controller.visionSystem.cameraHandler, builder.get_object("camerasMenuButton"))
    
    # Pilha do Gtk da página principal
    mainStack = builder.get_object("mainStack")
    
    # Adiciona a pilha a view de configuração da visão
    MainVisionView(self.__controller, self.__controller.visionSystem, self.__controller.world, mainStack)
    HLCView(self.__controller, self.__controller.world, mainStack)
    
    # Loop principal do Gtk
    Gtk.main()
