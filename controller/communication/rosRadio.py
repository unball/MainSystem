from controller.communication import Communication
from controller.control.speed_converter import speeds2motors
from controller.communication.rosHandler import RosHandler
from controller.control.NLC import SpeedPair
from main_system.msg import robot_msg
import rospy

class RosRadio(Communication):
  """Implementa a comunicação usando o ROS"""
  def __init__(self, world):
    super().__init__(world)
    
    #self.rh = RosHandler()
    #self.rh.runProcess("roscore") # Asserts roscore is running

    #rospy.init_node('Radio')
    #self.pub = rospy.Publisher("radio_topic", robot_msg, queue_size=1)
    #self.msg = robot_msg()

  def send(self, msg):
    """Envia a mensagem ao tópico `radio_topic` do ROS que será lido pelo rádio conectado ao computador."""
    return
    self.rh.runProcess("roscore") # Asserts roscore is running
    self.rh.runProcess("radioSerial") # Asserts radio is listening
    
    for i in range(self._world.n_robots):
        self.msg.MotorA[i], self.msg.MotorB[i] = speeds2motors(msg[i].v, msg[i].w)

    self.pub.publish(self.msg)
