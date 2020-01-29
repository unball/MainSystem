from controller.states import State
from controller.strategy.field import UVF
from controller.strategy.strategy import Strategy
from controller.world.robot import Robot
from controller.tools import norm, adjustAngle, ang, sat, unit
from controller.control import SpeedPair
from controller.control.UFC import UFC
from model.paramsPattern import ParamsPattern
from helpers import Mux
import numpy as np
import time
import copy
import json

class DebugHLC(ParamsPattern, State):
  """Estado de debug HLC. Executa a visão, define campos específicos para os robôs, passa um target ao controle de alto nível e envia sinal via rádio."""
  
  def __init__(self, controller):
    State.__init__(self, controller)
    ParamsPattern.__init__(self, "debugHLCState", {
      "manualControlSpeedV": 0,
      "manualControlSpeedW": 0,
      "enableManualControl": False,
      "selectedField": "UVF",
      "selectableFinalPoint": False,
      "runVision": True,
      "selectedHLCcontrol": 0
    })

    self.world = controller.world
    self.finalPoint = None
    self.currentFinalPoint = None
    self.currentField = None
    self.robot = self.world.robots[0]
    self.initialTime = time.time()
    self.t = time.time()
    self.finalPointIndex = 0
    self.loops = 0
    self.lastv = 0
    self.lastw = 0
    self.field = None
    self.strategy = Strategy(controller.world, controller.world.robots)
    
    
    self.v = 0

    self.debugData = {
      "time": [],
      "posX": [],
      "posY": [],
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

    # Sistemas de controle de alto nível suportados
    self.HLCs = Mux([UFC("debugHLC")], selected=self.getParam("selectedHLCcontrol"))

  def setFinalPoint(self, point):
    self.finalPoint = point

  def selectHLC(self, index):
    self.setParam("selectedHLCcontrol", index)
    self.HLCs.select(index)

  def changeFieldCondition(self):
    return (self.loops % 3 == 0) # self.finalPoint != self.currentFinalPoint or self.currentField != self.getParam("selectedField") or self.paramsChanged

  def changePointCondition(self, robot):
     return self.finalPoint is None or (not self.getParam("selectableFinalPoint") and norm(robot.pos, self.finalPoint) < 0.07)

  def saveData(self, filename):
    with open(filename, "w") as f:
      json.dump(self.debugData, f, indent=4)

  def simulate(self, v, w, dt=0.033, r=0.03, L=0.075, motorangaccelmax=15):
    v = v#self.lastv + sat(v-self.lastv, motorangaccelmax * r * dt / 2)
    w = w + (np.random.uniform() - 0.5) * (5 * v**2) / 0.5
    w = self.lastw + sat(w-self.lastw, motorangaccelmax * r * dt / L)
    if w != 0:
      R = v / w
      rc = (self.robot.x - R * np.sin(self.robot.th), self.robot.y + R * np.cos(self.robot.th))
      dth = w * dt
      x = rc[0] + abs(R) * np.cos(ang(rc, self.robot.pos) + dth)
      y = rc[1] + abs(R) * np.sin(ang(rc, self.robot.pos) + dth)
      th = adjustAngle(self.robot.th + dth)
    else:
      x = self.robot.x + v * dt * np.cos(self.robot.th)
      y = self.robot.y + v * dt * np.sin(self.robot.th)
      th = self.robot.th

    self.lastv = v
    self.lastw = w
    self.robot.update(x,y,th)

  def update(self):
    # Computa o tempo desde o último loop e salva
    dt = time.time()-self.t
    self.t = time.time()
    
    # Atualiza o mundo com a visão
    if self.getParam("runVision"):
      self._controller.visionSystem.update()
    
    
    # Robô de referência
    robot = self.robot
    
    # Condições para recalcular o ponto final
    if self.changePointCondition(robot) and not self.getParam("runVision"):
      finalPoints = [
       (-self.world.xmaxmargin, +self.world.ymaxmargin, 0),
       (+self.world.xmaxmargin, +self.world.ymaxmargin, -np.pi/2),
       (+self.world.xmaxmargin, -self.world.ymaxmargin, np.pi),
       (-self.world.xmaxmargin, -self.world.ymaxmargin, np.pi/2)
      ]
      self.finalPoint = finalPoints[self.finalPointIndex % 4]
      self.finalPointIndex += 1
      
    elif self.getParam("runVision"):
      if (self.world.ball.x == self.robot.x):
        k = 0
      else:
        k = (self.world.xmax-self.robot.x) / (self.world.ball.x-self.robot.x)
      yproj = k * (self.world.ball.y-self.robot.y) + self.robot.y
      if norm(self.world.ball.pose, self.robot.pose) < 0.15 and abs(yproj) <= 0.2:
        self.finalPoint = (self.world.xmax+0.2, 0, ang(self.world.ball.pose, (self.world.field_x_length/2, 0)))
      else: self.finalPoint = (*self.world.ball.pose[:2], ang(self.world.ball.pose, (self.world.field_x_length/2, 0)))
      #self.finalPoint = (*(np.array(self.finalPoint[:2]) + (0.05+sat(0.01**2/max(norm(self.world.ball.pose[:2], robot.pose),0.0001)**2, 1)) * unit(self.finalPoint[2])), self.finalPoint[2])

    # Condições para recomputar o campo
    if self.changeFieldCondition():

      self.currentFinalPoint = self.finalPoint

      # Atualiza o campo
      self.currentField = self.getParam("selectedField")

      if self.currentField == "UVF":
        self.field = UVF(self.finalPoint, self.world,
          h=self.world.getParam("UVF_h"),
          n=self.world.getParam("UVF_n"),
          avoidGoal=True,
          pointObstacles=self.world.enemyRobots,
          horRepSize=self.world.getParam("UVF_horRepSize"),
          horMinDist=self.world.getParam("UVF_horMinDist"),
          verRepSize=self.world.getParam("UVF_verRepSize"),
          verGoalSize=self.world.getParam("UVF_verGoalSize"),
          verMinDist=self.world.getParam("UVF_verMinDist"),
          ponRadius=self.world.getParam("UVF_ponRadius"),
          ponDistanceRadius=self.world.getParam("UVF_ponDistanceRadius"),
          ponMinAvoidanceAngle=self.world.getParam("UVF_ponMinAvoidanceAngle")       
        )

      self.strategy.run()
    
    # Define um campo
    #robot.field = self.field
    
    # Obtém o target instantâneo
    reference = robot.field.F(robot.pose)

    # Define um controle
    robot.controlSystem = self.HLCs.get()
    
    # Obtém velocidade angular e linear
    if self.getParam("enableManualControl"):
      speeds = [SpeedPair(self.getParam("manualControlSpeedV"), self.getParam("manualControlSpeedW"))]
    else:
      highLevelspeed = robot.controlSystem.actuate(reference, robot.pose, robot.field, self.lastw, dt)

      speeds = [SpeedPair(highLevelspeed.v,-highLevelspeed.w)]
    
    # Envia os dados via rádio
    if self.world.running:

      # Simula nova posição
      if not self.getParam("runVision"): self.simulate(speeds[0].v, speeds[0].w)

      # Alimenta dados de debug
      self.debugData["time"].append(time.time()-self.initialTime)
      self.debugData["posX"].append(robot.x)
      self.debugData["posY"].append(robot.y)
      self.debugData["posTh"].append(adjustAngle(robot.th))
      self.debugData["posThRef"].append(adjustAngle(reference))
      self.debugData["velLin"].append(speeds[0].v)
      self.debugData["visionLin"].append(robot.velmod)
      self.debugData["velAng"].append(speeds[0].w)
      self.debugData["visionAng"].append(robot.w)
      #self.debugData["distTarg"].append(norm(robot.pos, robot.controlSystem.currentTarget))
      self.debugData["distTarg"].append(robot.field.gamma(robot.pose))

      self._controller.communicationSystems.get().send(speeds)
    else: self._controller.communicationSystems.get().sendZero()

    # Mais dados de debug
    self.debugData["loopTime"] = (dt*1000)*0.1 + self.debugData["loopTime"]*0.9
    self.debugData["controlV"] = speeds[0].v
    self.debugData["controlW"] = speeds[0].w
    self.debugData["visionV"] = robot.velmod
    self.debugData["visionW"] = robot.w
    self.debugData["visionPose"] = (*robot.pos, robot.th*180/np.pi)
    
    time.sleep(max(0.033-(time.time()-self.t), 0))

    self.loops += 1
