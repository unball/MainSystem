from controller.states import State
from controller.strategy.trajectory import StraightLine, Circle, Dubins, Point, UnifiedVectorField
from controller.world.robot import Robot
from controller.tools import norm, adjustAngle, ang
from controller.control import SpeedPair
from controller.control.NLC import NLC
from controller.control.PID import PID
from controller.control.UFC import UFC, NLCUFC
from model.paramsPattern import ParamsPattern
from helpers import Mux
import numpy as np
import time
import copy
import json

class DebugHLC(ParamsPattern, State):
  """Estado de debug HLC. Executa a visão, define trajetórias específicas para os robôs, passa um target ao controle de alto nível e envia sinal via rádio."""
  
  def __init__(self, controller):
    State.__init__(self, controller)
    ParamsPattern.__init__(self, "debugHLCState", {
      "manualControlSpeedV": 0,
      "manualControlSpeedW": 0,
      "enableManualControl": False,
      "selectedTrajectory": "Dubins",
      "selectableFinalPoint": False,
      "runVision": True,
      "dubinsRadius": 0.15,
      "step": 0.20,
      "selectedHLCcontrol": 0,
      "UVF_h": 0.5,
      "UVF_n": 1,
      "UVF_runUnified": True,
      "UVF_showField": True
    })
    
    self.finalPoint = None
    self.currentFinalPoint = None
    self.currentTrajectory = None
    self.robot = Robot()
    self.initialTime = time.time()
    self.t = time.time()
    self.finalPointIndex = 0

    self.debugData = {
      "time": [],
      "posX": [],
      "posXRef": [],
      "posY": [],
      "posYRef": [],
      "posTh": [],
      "posThRef": [],
      "velLin": [],
      "visionLin": [],
      "velAng": [],
      "visionAng": [],
      "distTarg": [],
      "loopTime": 0,
      "controlV": 0,
      "controlW": 0,
      "visionV": 0,
      "visionW": 0,
      "visionPose": (0,0,0)
    }

    # Sistema de controle de médio nível
    self.vCtrl = PID("MLCV")
    self.wCtrl = PID("MLCW")

    # Sistemas de controle de alto nível suportados
    self.HLCs = Mux([NLCUFC("debugHLC"), UFC("debugHLC"), NLC("debugHLC")], selected=self.getParam("selectedHLCcontrol"))

  def setFinalPoint(self, point):
    self.finalPoint = point

  def selectHLC(self, index):
    self.setParam("selectedHLCcontrol", index)
    self.HLCs.select(index)

  def changeCondition(self):
    return self.finalPoint is None or self.finalPoint != self.currentFinalPoint or self.currentTrajectory != self.getParam("selectedTrajectory") or self.paramsChanged

  def saveData(self, filename):
    with open(filename, "w") as f:
      json.dump(self.debugData, f, indent=4, separators=(". ", " = "))

  def update(self):
    # Computa o tempo desde o último loop e salva
    dt = time.time()-self.t
    self.t = time.time()
    
    # Atualiza o mundo com a visão
    self._controller.visionSystem.update()
    
    # Se a flag de usar visão estiver habilitada, usa o robô no mundo
    if self.getParam("runVision"):
      self.robot = copy.deepcopy(self._controller.world.robots[0])
    
    # Robô de referência
    robot = self.robot
    robot.step = self.getParam("step")
    
    # Condições para recalcular a trajetória
    if self.changeCondition() or (not self.getParam("selectableFinalPoint") and norm(robot.pos, self.finalPoint) < 0.07):
      
      sign = np.sign(robot.x)
      if sign == 0: sign = 1
      
      # Condições para recalcular o ponto final
      if not self.getParam("selectableFinalPoint") or self.finalPoint is None:
        finalPoints = [
          (-self._controller.world.field_x_length/2*0.60, +self._controller.world.field_y_length/2*0.60, 0),
          (+self._controller.world.field_x_length/2*0.60, +self._controller.world.field_y_length/2*0.60, -np.pi/2),
          (+self._controller.world.field_x_length/2*0.60, -self._controller.world.field_y_length/2*0.60, np.pi),
          (-self._controller.world.field_x_length/2*0.60, -self._controller.world.field_y_length/2*0.60, np.pi/2)
        ]
        self.finalPoint = finalPoints[self.finalPointIndex % 4]
        self.finalPointIndex += 1
      self.currentFinalPoint = self.finalPoint
      
      # Atualiza a trajetória
      self.currentTrajectory = self.getParam("selectedTrajectory")

      if self.currentTrajectory == "Dubins":
        self.trajectory = Dubins(robot.pose, self.finalPoint, self.getParam("dubinsRadius"))
      elif self.currentTrajectory == "UVF":
        self.trajectory = UnifiedVectorField("debugHLC", robot.pose, self.finalPoint,
          h=self.getParam("UVF_h"),
          n=self.getParam("UVF_n"),
          runUnified=self.getParam("UVF_runUnified"),
          showField=self.getParam("UVF_showField"))
      elif self.currentTrajectory == "StraightLine":
        self.trajectory = StraightLine(robot.pose, self.finalPoint)
      elif self.currentTrajectory == "Circle":
        self.trajectory = Circle(robot.pose, self.finalPoint)
      elif self.currentTrajectory == "Point":
        self.trajectory = Point(self.finalPoint)
    
    # Define uma trajetória
    robot.trajectory = self.trajectory
    
    # Obtém o target instantâneo
    target = robot.trajectory.target(robot.pose, robot.step)

    # Define um controle
    robot.controlSystem = self.HLCs.get()
    
    # Obtém velocidade angular e linear
    if self.getParam("enableManualControl"):
      speeds = [SpeedPair(self.getParam("manualControlSpeedV"), self.getParam("manualControlSpeedW"))]
    else:
      highLevelspeed = robot.controlSystem.actuate(target, robot, False)
      v = self.vCtrl.actuate(highLevelspeed.v, robot.velmod, dt)
      w = self.wCtrl.actuate(highLevelspeed.w, robot.w, dt)

      speeds = [SpeedPair(v,w)]
    
    # Envia os dados via rádio
    if self._controller.world.running:

      # Alimenta dados de debug
      self.debugData["time"].append(time.time()-self.initialTime)
      self.debugData["posX"].append(robot.x)
      self.debugData["posXRef"].append(target[0])
      self.debugData["posY"].append(robot.y)
      self.debugData["posYRef"].append(target[1])
      self.debugData["posTh"].append(adjustAngle(robot.th))
      self.debugData["posThRef"].append(adjustAngle(target[2]))
      self.debugData["velLin"].append(speeds[0].v)
      self.debugData["visionLin"].append(robot.velmod)
      self.debugData["velAng"].append(speeds[0].w)
      self.debugData["visionAng"].append(robot.w)
      self.debugData["distTarg"].append(norm(robot.pos, target))

      self._controller.communicationSystems.get().send(speeds)
    else: self._controller.communicationSystems.get().sendZero()

    # Mais dados de debug
    self.debugData["loopTime"] = (dt*1000)*0.1 + self.debugData["loopTime"]*0.9
    self.debugData["controlV"] = speeds[0].v
    self.debugData["controlW"] = speeds[0].w
    self.debugData["visionV"] = robot.velmod
    self.debugData["visionW"] = robot.w
    self.debugData["visionPose"] = (*robot.pos, robot.th*180/np.pi)
    
    
