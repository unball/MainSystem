from controller.states import State
from controller.strategy.field import UVF
from controller.strategy.strategy import Strategy
from controller.world.robot import Robot
from controller.tools import norm, adjustAngle, ang, sat, unit, angError
from controller.control import SpeedPair
from controller.control.UFC import UFC
from controller.tools.simulator import simulate, simulateBall
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
    """Referência para o mundo"""

    self.robot = self.world.robots[0]
    """Referência para o robô 0"""

    self.initialTime = time.time()
    """Tempo inicial a ser usado como origem de tempo dos dados de debug"""

    self.firstLoopRunning = True
    self.replayInitialTime = 0

    self.t = time.time()
    """Tempo do início do loop anterior"""

    self.loops = 0
    """Loops executados"""

    self.strategy = Strategy(controller.world, controller.world.robots)
    """Instância da estratégia"""

    self.debugData = {
      "time": [],
      "posX": [],
      "posY": [],
      "posTh": [],
      "posThRef": [],
      "posThErr": [],
      "velLin": [],
      "visionLin": [],
      "velAng": [],
      "visionAng": [],
      "velBallX": [],
      "velBallY": [],
      "velBallMod": [],
      "accBallX": [],
      "accBallY": [],
      "accBallMod": [],
      "velRobotX": [],
      "velRobotY": [],
      "velRobotMod": [],
      "replayData": {"time": [], "robot": [], "ball": []},
      "loopTime": 0,
      "controlV": 0,
      "controlW": 0,
      "visionV": 0,
      "visionW": 0,
      "visionPose": (0,0,0)
    }
    """Dados de debug"""

    self.HLCs = Mux([UFC("debugHLC")], selected=self.getParam("selectedHLCcontrol"))
    """Sistemas de controle de alto nível suportados"""

  def setFinalPoint(self, point):
    """Atualiza a posição da bola"""
    self.world.ball.update(*point)

  @property
  def finalPoint(self):
    return self.world.ball.pose

  def selectHLC(self, index):
    """Muda o controle alto nível selecionado"""
    self.setParam("selectedHLCcontrol", index)
    self.HLCs.select(index)

  def runStrategyCondition(self):
    """Condição para rodar a estratégia"""
    return (self.loops % 1 == 0)
  
  def saveData(self, filename):
    """Salva os dados de debug no arquivo `filename`"""
    with open(filename, "w") as f:
      json.dump(self.debugData, f, indent=4)

  def appendDebugData(self, reference, speeds, dt):
    """Alimenta os dados de debug com o estado atual das variáveis"""

    if self.world.running:
      # Alimenta dados de debug
      if self.initialTime is None: self.initialTime = time.time()
      self.debugData["time"].append(time.time()-self.initialTime)
      self.debugData["posX"].append(self.robot.x)
      self.debugData["posY"].append(self.robot.y)
      self.debugData["posTh"].append(adjustAngle(self.robot.th))
      self.debugData["posThRef"].append(adjustAngle(reference))
      self.debugData["posThErr"].append(angError(reference, self.robot.th))
      self.debugData["velLin"].append(speeds[0].v)
      self.debugData["visionLin"].append(self.robot.velmod)
      self.debugData["velAng"].append(speeds[0].w)
      self.debugData["visionAng"].append(self.robot.w)
      self.debugData["velBallX"].append(self.world.ball.vel[0])
      self.debugData["velBallY"].append(self.world.ball.vel[1])
      self.debugData["velBallMod"].append(self.world.ball.velmod)
      self.debugData["accBallX"].append(self.world.ball.acc[0])
      self.debugData["accBallY"].append(self.world.ball.acc[1])
      self.debugData["accBallMod"].append(self.world.ball.accmod)
      self.debugData["velRobotX"].append(self.robot.vel[0])
      self.debugData["velRobotY"].append(self.robot.vel[1])
      self.debugData["velRobotMod"].append(self.robot.velmod)

      if self.firstLoopRunning:
        for k in self.debugData["replayData"]: self.debugData["replayData"][k].clear()
        self.firstLoopRunning = False
        self.replayInitialTime = time.time()
      
      self.debugData["replayData"]["time"].append(time.time()-self.replayInitialTime)
      self.debugData["replayData"]["robot"].append(copy.deepcopy(self.robot))
      self.debugData["replayData"]["ball"].append(copy.deepcopy(self.world.ball))

    else: self.firstLoopRunning = True

    # Mais dados de debug
    self.debugData["loopTime"] = (dt*1000)*0.1 + self.debugData["loopTime"]*0.9
    self.debugData["controlV"] = speeds[0].v
    self.debugData["controlW"] = speeds[0].w
    self.debugData["visionV"] = self.robot.velmod
    self.debugData["visionW"] = self.robot.w
    self.debugData["visionPose"] = (*self.robot.pos, self.robot.th*180/np.pi)
  
  def update(self):
    """Função de loop do estado debugHLC"""

    # Computa o tempo desde o último loop e salva
    dt = time.time()-self.t
    self.t = time.time()
    
    # Atualiza o mundo com a visão
    if self.getParam("runVision"):
      self._controller.visionSystem.update()

    # Condições para rodar a estratégia
    if self.runStrategyCondition():
      self.strategy.run()
    
    # Obtém o target instantâneo
    reference = self.robot.field.F(self.robot.pose)

    # Atualiza o erro angular do robô
    self.robot.lastAngError = angError(reference, self.robot.th)

    # Define um controle
    self.robot.controlSystem = self.HLCs.get()
    
    # Controle manual
    if self.getParam("enableManualControl"):
      speeds = [SpeedPair(self.getParam("manualControlSpeedV"), self.getParam("manualControlSpeedW"))]

    # Controle de alto nível
    else:
      highLevelspeed = self.robot.controlSystem.actuate(reference, self.robot.pose, self.robot.field, self.robot.dir, self.robot.gammavels, self.robot.vref)
      speeds = [SpeedPair(highLevelspeed.v,-highLevelspeed.w)]

    # Adiciona dados de debug para gráficos e para salvar
    self.appendDebugData(reference, speeds, dt)

    if self.world.running:
      # Envia mensagem ao robô
      if self.getParam("runVision"): self._controller.communicationSystems.get().send(speeds)

      # Simula nova posição
      else:
        simulate(self.robot, speeds[0].v, -speeds[0].w)
        simulateBall(self.world.ball)
    
    # Envia zero para o robô
    else: self._controller.communicationSystems.get().sendZero()

    # Garante que o tempo de loop é de no mínimo 33ms
    time.sleep(max(0.033-(time.time()-self.t), 0))

    # Incrementa o número de loops
    self.loops += 1
