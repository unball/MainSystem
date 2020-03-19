from gi.repository import Gtk
from pkg_resources import resource_filename

class CameraHandlerView():
  """Classe que gerencia a view do gerenciador de câmeras"""
  
  def __init__(self, controller, cameraHandler, menuButton):
    self.__cameraHandler = cameraHandler
    """Mantém referência ao gerenciador de câmeras"""
    
    self.__menuButton = menuButton
    """Botão de câmera"""
    
    self.__controller = controller
    """Mantém referência ao controller principal"""
    
    self.__listBoxCameras = set()
    """Lista de câmeras que já foram adicionadas a lista `cameraPopoverCamList`"""
    
    # Carrega os elementos de interface gráfica
    builder = Gtk.Builder.new_from_file(resource_filename(__name__, "cameras.ui"))
    
    self.__cameraPopover = builder.get_object("cameraPopover")
    """Referência ao objeto de interface gráfica `cameraPopover`"""
    
    self.__cameraPopoverCamList = builder.get_object("cameraPopoverCamList")
    """Referência ao objeto de interface gráfica `cameraPopoverCamList`, que contém a lista de câmeras"""
    
    self.__cameraPopoverAdjustment = builder.get_object("cameraPopoverAdjustment")
    """Referência ao objeto de interface gráfica `cameraPopoverAdjustment`, que controla a escala"""

    self.__cameraFrameParamsUpdate = builder.get_object("cameraParamsUpdate")
    """Referência ao objeto de interface gráfica `cameraParamsUpdate`, que quando clicado atualiza os parâmetros da câmera"""

    self.__cameraFrameWidthEntry = builder.get_object("cameraPopoverWidth")
    """Referência ao objeto de interface gráfica `cameraPopoverWidth`, que controla a largura do frame na fonte"""

    self.__cameraFrameHeightEntry = builder.get_object("cameraPopoverHeight")
    """Referência ao objeto de interface gráfica `cameraPopoverWidth`, que controla a altura do frame na fonte"""

    self.__cameraFrameFPSEntry = builder.get_object("cameraPopoverFPS")
    """Referência ao objeto de interface gráfica `cameraPopoverWidth`, que controla a taxa de frames na fonte"""

    self.__cameraFrameFormatCombo = builder.get_object("cameraPopoverFormat")
    """Referência ao objeto de interface gráfica `cameraPopoverFormat`, que controla o formato"""
    
    # Faz com que quando o botão é clicado apareça o cameraPopover
    self.__menuButton.set_popover(self.__cameraPopover)
    
    # Carrega o valor inicial da escala
    self.__cameraPopoverAdjustment.set_value(cameraHandler.getScale())
    self.sync_values()
    
    # Adiciona a câmera "padrão" que é a imagem padrão
    self.add_to_list_box(self.__cameraPopoverCamList, -1, "Padrão")
    
    # Adiciona os sinais
    self.__menuButton.connect("toggled", self.list_cameras, self.__cameraPopoverCamList)
    self.__cameraPopoverAdjustment.connect("value-changed", self.update_camera_scale)
    self.__cameraPopoverCamList.connect("row-selected", self.select_camera)
    self.__cameraFrameParamsUpdate.connect("clicked", self.update_camera_params)
    
    
  def update_camera_scale(self, widget):
    """Agenda uma mudança de escala de acordo com o valor selecionado pelo `cameraPopoverAdjustment`"""
    self.__controller.addEvent(self.__cameraHandler.setScale, widget.get_value())

  def sync_values(self):
    self.__cameraFrameWidthEntry.set_text(str(self.__cameraHandler.getParam("frame_width")))
    self.__cameraFrameHeightEntry.set_text(str(self.__cameraHandler.getParam("frame_height")))
    self.__cameraFrameFPSEntry.set_text(str(self.__cameraHandler.getParam("frame_fps")))
    self.__cameraFrameFormatCombo.set_active_id(self.__cameraHandler.getParam("frame_format"))

  def update_camera_params(self, widget):
    self.__controller.addEvent(self.__cameraHandler.setCameraParams, {
      "frame_width": int(self.__cameraFrameWidthEntry.get_text()),
      "frame_height": int(self.__cameraFrameHeightEntry.get_text()),
      "frame_fps": int(self.__cameraFrameFPSEntry.get_text()),
      "frame_format": self.__cameraFrameFormatCombo.get_active_id()
    }, run_when_done_with_glib=(self.sync_values,))

      
  def add_to_list_box(self, widget_list, index, name):
    """Adiciona a um `widget_list` uma câmera com índice `index` e com nome `name`. Cria com ela selecionada caso o índice for igual ao índice de câmera atualmente selecionado."""
    row = Gtk.ListBoxRow()
    row.index = index
    row.add(Gtk.Label(name))
    row.set_size_request(150,30)
    widget_list.insert(row,-1)
    widget_list.show_all()
    if index == self.__cameraHandler.getCamera(): widget_list.select_row(row)
    
  def update_list_box(self, widget_list):
    """Adiciona novas câmeras ao `cameraPopoverCamList`"""
    for camera in sorted(set(self.__cameraHandler.getCameras()).difference(self.__listBoxCameras)):
      self.__listBoxCameras.add(camera)
      self.add_to_list_box(widget_list, int(camera.split("video")[1]), camera)
  
  def list_cameras(self, widget, widget_list):
    """Agenda uma mudança para o gerenciador de câmeras detectar novas câmeras, depois de detectadas, adiciona as novas ao `cameraPopoverCamList`"""
    self.__controller.addEvent(self.__cameraHandler.updateCameras, run_when_done_with_glib=(self.update_list_box, widget_list))
    
  def select_camera(self, widget, widget_selected):
    """Agenda uma mudança para o gerenciador de câmeras usar a câmera selecionada no `cameraPopoverCamList`"""
    if widget_selected is None: return
    self.__controller.addEvent(self.__cameraHandler.setCamera, widget_selected.index)
  
