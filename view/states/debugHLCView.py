from gi.repository import Gtk, GLib
from pkg_resources import resource_filename
from view.tools.stackSelector import StackSelector
from view.strategy.highLevelRenderer import HighLevelRenderer
from view.tools.viewMux import ViewMux
from controller.states.debugHLC import DebugHLC
from controller.tools import norm
from helpers import LoopThread
from view.tools.plotter import Plotter
import numpy as np
import time

class DebugHLCView(LoopThread, StackSelector):
  """Classe que gerencia a view de depurador do controle de alto nível"""

  def __init__(self, controller, world, stack):
    self.__controller = controller
    self.__world = world
    self.__controllerState = None
    self.limit = 250
    LoopThread.__init__(self, self.view_worker)
    StackSelector.__init__(self, stack, "configHLC", "HLC")

  def ui(self):
    # Carrega os elementos estáticos
    builder = Gtk.Builder.new_from_file(resource_filename(__name__, "debugHLCView.ui"))

    # Elementos internos
    mainBox = builder.get_object("HLCBox")
    renderContainer = builder.get_object("HLCRender")
    playPause = builder.get_object("HLCPlayPause")
    saveData = builder.get_object("HLCSaveData")
    self.dubinsRadiusEntry = builder.get_object("HLC_dubinsRadius")
    self.stepEntry = builder.get_object("HLC_step")
    self.UVF_h = builder.get_object("HLC_UVF_h")
    self.UVF_n = builder.get_object("HLC_UVF_n")
    self.UVF_unified = builder.get_object("HLC_UVF_unified")
    self.UVF_showField = builder.get_object("HLC_UVF_showField")
    self.selectableFinalPoint = builder.get_object("HLCSelectableFinalPoint")

    self.HLCcontrolList = ViewMux(self.__controller)
    HLCcontrolListBox = builder.get_object("HLCControlChooserBox")
    HLCcontrolListBox.pack_end(self.HLCcontrolList, True, True, 0)

    self.trajectoryList = builder.get_object("HLCTrajectoryChooser")
    self.manualControl = builder.get_object("HLCSwitchManualControl")
    self.manualControlLin = builder.get_object("manualControlLin")
    self.manualControlAng = builder.get_object("manualControlAng")
    self.useVisionButton = builder.get_object("HLCUseVision")
    self.MLCVEnableButton = builder.get_object("HLCMLCVenable")
    self.MLCVadjs = {
      "kp": builder.get_object("MLC_vKp"),
      "ki": builder.get_object("MLC_vKi"),
      "kd": builder.get_object("MLC_vKd")
    }
    self.MLCWEnableButton = builder.get_object("HLCMLCWenable")
    self.MLCWadjs = {
      "kp": builder.get_object("MLC_wKp"),
      "ki": builder.get_object("MLC_wKi"),
      "kd": builder.get_object("MLC_wKd")
    }
    self.loopTimeLabel = builder.get_object("HLCLoopTime")
    self.controlVLabel = builder.get_object("HLCcontrolV")
    self.controlWLabel = builder.get_object("HLCcontrolW")
    self.visionVLabel = builder.get_object("HLCvisionV")
    self.visionWLabel = builder.get_object("HLCvisionW")
    self.visionPoseLabel = builder.get_object("HLCvisionPose")

    self.__renderer = HighLevelRenderer(self.__world, robotsGetter=self.robotsGetter, on_click=self.on_click, on_scroll=self.on_scroll)
    """Instancia o renderizador, ele é do tipo GtkFrame"""

    # Adiciona o renderizador ao GtkBox
    renderContainer.add(self.__renderer)

    # Adiciona os gráficos
    self.__plots = {
      "PlotPosX": (Plotter(), ("posX","posXRef")),
      "PlotPosY": (Plotter(), ("posY","posYRef")),
      "PlotPosTh": (Plotter(), ("posTh","posThRef")),
      "PlotVelLin": (Plotter(), ("velLin","visionLin")),
      "PlotVelAng": (Plotter(), ("velAng","visionAng")),
      "PlotDistTarg": (Plotter(), ("distTarg",))
    }
    for el in self.__plots: builder.get_object(el).add(self.__plots[el][0])

    # Liga os sinais
    playPause.connect("toggled", self.playPause)
    saveData.connect("clicked", self.saveData)
    self.dubinsRadiusEntry.connect("value-changed", self.setHLCParam, "dubinsRadius")
    self.UVF_h.connect("value-changed", self.setHLCParam, "UVF_h")
    self.UVF_n.connect("value-changed", self.setHLCParam, "UVF_n")
    self.UVF_unified.connect("state-set", self.setHLCParam_state_set, "UVF_runUnified")
    self.UVF_showField.connect("state-set", self.setHLCParam_state_set, "UVF_showField")
    self.stepEntry.connect("value-changed", self.setHLCParam, "step")
    self.selectableFinalPoint.connect("state-set", self.setHLCParam_state_set, "selectableFinalPoint")
    self.trajectoryList.connect("row-activated", self.trajectoryChooser)
    self.manualControl.connect("state-set", self.setHLCParam_state_set, "enableManualControl")
    self.manualControlLin.connect("value-changed", self.setHLCParam, "manualControlSpeedV")
    self.manualControlAng.connect("value-changed", self.setHLCParam, "manualControlSpeedW")
    self.useVisionButton.connect("state-set",  self.setHLCParam_state_set, "runVision")
    self.MLCVEnableButton.connect("state-set", self.setMLCVparam_state_set, "enable")
    for key in self.MLCVadjs:
      self.MLCVadjs[key].connect("value-changed", self.setMLCVparam, key)
    self.MLCWEnableButton.connect("state-set", self.setMLCWparam_state_set, "enable")
    for key in self.MLCWadjs:
      self.MLCWadjs[key].connect("value-changed", self.setMLCWparam, key)

    return mainBox

  def on_click(self, p):
    finalPoint = (*p, self.__controllerState.finalPoint[2])
    self.__controller.addEvent(self.__controllerState.setFinalPoint, finalPoint)

  def on_scroll(self, mouse, delta):
    finalPoint = (*self.__controllerState.finalPoint[:2], self.__controllerState.finalPoint[2]+delta*0.1)
    if norm(finalPoint, mouse) < 0.07:
      self.__controller.addEvent(self.__controllerState.setFinalPoint, finalPoint)

  def view_worker(self):
    GLib.idle_add(self.loopTimeLabel.set_text, "{:.2f} ms".format(self.__controllerState.debugData["loopTime"]))
    GLib.idle_add(self.controlVLabel.set_text, "{:.2f} m/s".format(self.__controllerState.debugData["controlV"]))
    GLib.idle_add(self.controlWLabel.set_text, "{:.2f} rad/s".format(self.__controllerState.debugData["controlW"]))
    GLib.idle_add(self.visionVLabel.set_text, "{:.2f} m/s".format(self.__controllerState.debugData["visionV"]))
    GLib.idle_add(self.visionWLabel.set_text, "{:.2f} rad/s".format(self.__controllerState.debugData["visionW"]))
    GLib.idle_add(self.visionPoseLabel.set_text, "x: {:.2f} m\ty: {:.2f} m\tth: {:6.2f} º".format(*self.__controllerState.debugData["visionPose"]))
    for key in self.__plots:
      GLib.idle_add(self.__plots[key][0].set_data, *self.getData(*self.__plots[key][1]))
      GLib.idle_add(self.__plots[key][0].queue_draw)
    time.sleep(0.03)

  def getData(self, *dataNames):
    if self.__controllerState is None: return [],[]
    xdata = []
    ydata = []
    for name in dataNames:
      xdata.append(self.__controllerState.debugData["time"][-self.limit:])
      ydata.append(self.__controllerState.debugData[name][-self.limit:])
    return xdata, ydata

  def robotsGetter(self):
    if self.__controllerState is None: return []
    return [self.__controllerState.robot]

  def getRowByName(self, listBox, key):
    for i in range(5):
      row = listBox.get_row_at_index(i)
      if row is None: return
      if row.get_name() == key: return row

  def playPause(self, widget):
    self.__controller.addEvent(self.__world.setRunning, widget.get_active())

  def saveData(self, widget):
    dialog = Gtk.FileChooserDialog("Salvar arquivo", None, Gtk.FileChooserAction.SAVE,
      (Gtk.STOCK_CANCEL,
       Gtk.ResponseType.CANCEL,
       Gtk.STOCK_SAVE,
       Gtk.ResponseType.ACCEPT))

    dialog.connect("response", self.saveDataResponse)
    dialog.show()

  def saveDataResponse(self, dialog, response):
    if response == Gtk.ResponseType.ACCEPT:
      self.__controller.addEvent(self.__controllerState.saveData, dialog.get_filename())
    dialog.destroy()

  def setHLCParam(self, widget, key):
    self.__controller.addEvent(self.__controllerState.setParam, key, widget.get_value())

  def setHLCParam_state_set(self, widget, state, key):
    self.__controller.addEvent(self.__controllerState.setParam, key, state)

  def trajectoryChooser(self, widget, row):
    self.__controller.addEvent(self.__controllerState.setParam, "selectedTrajectory", row.get_name())

  def setMLCVparam(self, widget, key):
    self.__controller.addEvent(self.__controllerState.vCtrl.setParam, key, widget.get_value())

  def setMLCVparam_state_set(self, widget, state, key):
    self.__controller.addEvent(self.__controllerState.vCtrl.setParam, key, state)

  def setMLCWparam(self, widget, key):
    self.__controller.addEvent(self.__controllerState.wCtrl.setParam, key, widget.get_value())

  def setMLCWparam_state_set(self, widget, state, key):
    self.__controller.addEvent(self.__controllerState.wCtrl.setParam, key, state)

  def updateParam(self, widget, controlSystem, key):
    self.__controller.addEvent(controlSystem.setParam, key, widget.get_value())

  def on_select(self, widget):
    self.__controllerState = DebugHLC(self.__controller)

    self.HLCcontrolList.setMux(self.__controllerState.HLCs)

    # Define valores padrão
    self.selectableFinalPoint.set_state(self.__controllerState.getParam("selectableFinalPoint"))
    self.dubinsRadiusEntry.set_value(self.__controllerState.getParam("dubinsRadius"))
    self.UVF_h.set_value(self.__controllerState.getParam("UVF_h"))
    self.UVF_n.set_value(self.__controllerState.getParam("UVF_n"))
    self.UVF_unified.set_state(self.__controllerState.getParam("UVF_runUnified"))
    self.UVF_showField.set_state(self.__controllerState.getParam("UVF_showField"))
    self.stepEntry.set_value(self.__controllerState.getParam("step"))
    self.trajectoryList.select_row(self.getRowByName(self.trajectoryList, self.__controllerState.getParam("selectedTrajectory")))
    self.manualControl.set_state(self.__controllerState.getParam("enableManualControl"))
    self.manualControlLin.set_value(self.__controllerState.getParam("manualControlSpeedV"))
    self.manualControlAng.set_value(self.__controllerState.getParam("manualControlSpeedW"))
    self.useVisionButton.set_state(self.__controllerState.getParam("runVision"))
    self.MLCVEnableButton.set_state(self.__controllerState.vCtrl.getParam("enable"))
    for key in self.MLCVadjs:
      self.MLCVadjs[key].set_value(self.__controllerState.vCtrl.getParam(key))
    self.MLCWEnableButton.set_state(self.__controllerState.wCtrl.getParam("enable"))
    for key in self.MLCWadjs:
      self.MLCWadjs[key].set_value(self.__controllerState.wCtrl.getParam(key))

    self.__controller.addEvent(self.__controller.setState, self.__controllerState)
    self.__renderer.start()
    self.start()

  def on_deselect(self, widget):
    self.__renderer.stop()
    self.stop()
    self.__controller.addEvent(self.__controller.unsetState)
