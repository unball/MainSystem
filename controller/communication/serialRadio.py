from controller.communication import Communication
from controller.tools.speedConverter import speeds2motors
from model.paramsPattern import ParamsPattern
import serial
import time

class SerialRadio(ParamsPattern, Communication):
  """Implementa a comunicação usando simplesmente a interface serial"""
  def __init__(self, world):
    ParamsPattern.__init__(self, "SerialRadio", {}, name="Serial+Rádio")
    Communication.__init__(self, world)

    self.serial = None

  def send(self, msg, waitack=True):
    """Envia a mensagem via barramento serial em `/dev/ttyUSB0`."""
    try:
      if self.serial is None:
        self.serial = serial.Serial('/dev/ttyUSB0', 115200)
    except:
      print("Falha ao abrir serial")
      return

    # Início da mensagem
    message = bytes("BBB", encoding='ascii')

    # Checksum
    checksum = 0

    # Adiciona as cinco velocidades
    for i in range(5):

      # Converte para velocidade nos motores
      if i < len(msg):
        va, vb = speeds2motors(msg[i].v, msg[i].w)
      else:
        va, vb = (0,0)

      # Concatena em bytes
      message += (va).to_bytes(2,byteorder='little', signed=True)
      message += (vb).to_bytes(2,byteorder='little', signed=True)

      # Computa o checksum
      checksum += va + vb

    # Concatena com o checksum
    message += (checksum % 65536).to_bytes(2,byteorder='little')

    # Envia
    try:
      self.serial.write(message)
      if waitack:
        result = self.serial.readline()
        if result.decode() != "OK\r\n": print(result)
    except:
      print("Falha ao enviar")
      self.serial = None
