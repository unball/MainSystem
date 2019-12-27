from controller.communication import Communication
from controller.tools.speedConverter import speeds2motors
from controller.communication.rosHandler import RosHandler
from controller.communication.rosHandler import roscorechecker
from controller.control.NLC import SpeedPair
from model.paramsPattern import ParamsPattern
from main_system.msg import robot_msg
from gi.repository import GLib
import rospy

class RosRadio(ParamsPattern, Communication):
  """Implementa a comunicação usando o ROS"""
  def __init__(self, world):
    ParamsPattern.__init__(self, "ROSRadio", {"enable": False}, name="ROS+Rádio", properties={"enable": {"name": "Habilitar ROS"}})
    Communication.__init__(self, world)
    
    self.rh = RosHandler()

    self.pub = rospy.Publisher("radio_topic", robot_msg, queue_size=1)
    self.msg = robot_msg()

  def setParam(self, key, value):
    if key == "enable": self.setEnable(value)
    else: ParamsPattern.setParam(self, key, value)

  def setEnable(self, flag):
    """Atualiza o estado de habilitado e fecha os componentes do ROS caso esteja desabilitando"""
    if flag is False:
      ParamsPattern.setParam(self, "enable", False)
      self.rh.terminateAll()
    else:
      GLib.idle_add(self.initProcesses)

  def initProcesses(self):
    self.rh.runProcess("roscore") # Asserts roscore is running
    while not roscorechecker(): pass
    rospy.init_node('radio')
    self.rh.runProcess("radioSerial") # Asserts radio is listening
    ParamsPattern.setParam(self, "enable", True)

  def send(self, msg):
    """Envia a mensagem ao tópico `radio_topic` do ROS que será lido pelo rádio conectado ao computador."""
    
    if not self.getParam("enable"): return

    for i in range(len(msg)):
        self.msg.MotorA[i], self.msg.MotorB[i] = speeds2motors(msg[i].v, msg[i].w)

    self.pub.publish(self.msg)
