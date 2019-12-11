from threading import Thread

class ViewThreads():
  """Classe que implementa uma lista de threads e permite finalizar todas de uma só vez"""
  def __init__(self):
    self.__threads = []
    
  def stop(self):
    """Este método finaliza todas as threads registradas de uma só vez"""
    for thread in self.__threads: thread.stop()
  
  def register(self, thread):
    """Este método registra uma thread"""
    self.__threads.append(thread)


class ViewLoopThread():
  """Classe que implementa uma thread que fica executando repetidamente uma função `worker` passada."""
  def __init__(self, worker):
    self.__worker = worker
    
  def start(self):
    """Método que inicia a execução da thread de loop"""
    self.__thread = Thread(target=self.loop)
    self.__running = True
    self.__thread.start()
  
  def loop(self):
    """Método que mantém o loop em execução e é usado como fluxo base para a Thread da biblioteca threading"""
    while self.__running:
      self.__worker()
  
  def stop(self):
    """Método que muda a flag de executando para `False`, fazendo com que a thread de loop pare"""
    self.__running = False
