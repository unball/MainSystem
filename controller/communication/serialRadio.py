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
        self.serial.timeout = 0.005
    except:
      print("Falha ao abrir serial")
      return

    # Início da mensagem
    message = bytes("BBB", encoding='ascii')

    # Checksum
    checksum = 0

    # Vetor de dados
    data = [0] * 10

    # Adiciona as velocidades ao vetor de dados
    for i,m in enumerate(msg):

      # Converte para velocidade nos motores
      va, vb = speeds2motors(m.v, m.w)

      # Coloca no vetor de dados
      data[i] = va
      data[i+5] = vb

      # Computa o checksum
      checksum += va + vb

    # Concatena o vetor de dados à mensagem
    for v in data: message += (v).to_bytes(2,byteorder='little', signed=True)

    # Concatena com o checksum
    message += (checksum).to_bytes(2,byteorder='little', signed=True)

    # Envia
    try:
      self.serial.write(message)
      if waitack:
        response = self.serial.readline().decode()
        result = list(map(lambda x:int(x), response.replace("\n","").split("\t")))
        if len(result) != 3: print("ACK de tamanho errado")
        else:
          if result[0] != checksum or result[1] != data[0] or result[2] != data[5]:
            print("Enviado:\t" + str(checksum) + "\t" + str(data[0]) + "\t" + str(data[5]))
            print("ACK:\t\t" + response)
    except:
      print("Falha ao enviar")
      self.serial = None
