from controller.states import State
from controller.strategy.trajectory import StraightLine, Dubins
import numpy as np
import time

class Individuo():
  def __init__(self, params):
    self.params = params
    self.adequacao = 0
    
  @property
  def ka(self):
    return self.params[0]
    
  @property
  def kg(self):
    return self.params[1]
    
  @property
  def kp(self):
    return self.params[2]

class AG():
  def __init__(self, size):
    self.currentIndividuo = 0
    
    self.weights = [10,500,5]
    self.min = [4, 0, 7]
    
    self.initialGen = [{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
{"ka": 1.1172688438443967, "kg": 109.4501515549437, "kp": 5.9396152856884},
{"ka": 6.699208305468581, "kg": 382.8649280089019, "kp": 2.377935126960727},
{"ka": 5.837680521958922, "kg": 133.61676185098975, "kp": 4.555786188293416},
{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
{"ka": 5.837680521958922, "kg": 133.61676185098975, "kp": 4.555786188293416},
{"ka": 5.617773757461554, "kg": 14.683103375551232, "kp": 5.9396152856884},
{"ka": 5.617773757461554, "kg": 14.683103375551232, "kp": 5.9396152856884},
{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
{"ka": 5.617773757461554, "kg": 14.683103375551232, "kp": 5.9396152856884},
{"ka": 5.837680521958922, "kg": 14.683103375551232, "kp": 5.9396152856884},
{"ka": 4.7270998549824705, "kg": 118.99659632111248, "kp": 5.870844036094553},
{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 2.5297938123647503},
{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
{"ka": 5.837680521958922, "kg": 133.61676185098975, "kp": 4.555786188293416},
{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 9.385447667773759},
{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553}]
    
    self.individuos = [Individuo([self.weights[0]*np.random.rand()+self.min[0], self.weights[1]*np.random.rand()+self.min[1], self.weights[2]*np.random.rand() + self.min[2]]) for i in range(size)]
    #self.individuos = [Individuo([self.initialGen[i]["ka"], self.initialGen[i]["kg"], self.initialGen[i]["kp"]]) for i in range(size)]
  
    
  def setAdequacao(self, valor):
    self.individuos[self.currentIndividuo].adequacao = valor
    self.currentIndividuo = self.currentIndividuo + 1
    
    # Fim do experimento com todos os indivíduos
    if self.currentIndividuo == len(self.individuos):
      self.reproduzir()
      self.currentIndividuo = 0
      
  def getIndividuo(self):
    return self.individuos[self.currentIndividuo]
    
  def corrigirAdequacao(self):
    maiorAdequacao = max(self.individuos, key=lambda x: x.adequacao).adequacao
    soma = 0
    for i in range(len(self.individuos)):
      self.individuos[i].adequacao = maiorAdequacao-self.individuos[i].adequacao
      soma = soma + self.individuos[i].adequacao
    return soma
    
    
  def encontrarIndividuo(self, regioes, prob):
    for i in range(len(self.individuos)):
      if regioes[i][0] <= prob and regioes[i][1] >= prob:
        return i
    return None
      
  def reproduzir(self):
    somaAdequacao = self.corrigirAdequacao()
    
    print("Resultado indivíduos:")
    for individuo in sorted(self.individuos, key=lambda x:-x.adequacao):
      print("Adequação: {3}\tka: {0}\tkg: {1}\tkp: {2}".format(individuo.ka, individuo.kg, individuo.kp, individuo.adequacao))
      
    listaRegiaoProb = []
    probAnt = 0
    for i in range(len(self.individuos)):
      prob = self.individuos[i].adequacao/somaAdequacao
      listaRegiaoProb.append((probAnt,probAnt+prob))
      probAnt = probAnt+prob
      
    print(listaRegiaoProb)
    
    pais = []
    maes = []
    for i in range(len(self.individuos)//2):
      pais.append(self.individuos[self.encontrarIndividuo(listaRegiaoProb, np.random.rand())])
      maes.append(self.individuos[self.encontrarIndividuo(listaRegiaoProb, np.random.rand())])
    
    novosIndividuos = []
    for i in range(len(self.individuos)//2):
      pc = np.random.rand()
      if pc > 0.7:
        locus = np.random.randint(3)+1
        novosIndividuos.append(Individuo(pais[i].params[:locus]+maes[i].params[locus:3]))
        novosIndividuos.append(Individuo(maes[i].params[:locus]+pais[i].params[locus:3]))
      else:
        novosIndividuos.append(Individuo(pais[i].params))
        novosIndividuos.append(Individuo(maes[i].params))
      
      for j in range(len(novosIndividuos[-1].params)):
        pm1 = np.random.rand()
        pm2 = np.random.rand()
        if pm1 > 0.97:
          novosIndividuos[-1].params[j] = self.weights[j] * np.random.rand() + self.min[j]
        if pm2 > 0.97:
          novosIndividuos[-2].params[j] = self.weights[j] * np.random.rand() + self.min[j]
    
    print("Novos indivíduos:")
    for individuo in novosIndividuos:
      print("ka: {0}\tkg: {1}\tkp: {2}".format(individuo.ka, individuo.kg, individuo.kp))
    
    self.individuos = novosIndividuos

class DebugHLC(State):
  """Estado de debug HLC. Executa a visão, define trajetórias específicas para os robôs, passa um target ao controle de alto nível e envia sinal via rádio."""
  
  def __init__(self, controller):
    super().__init__(controller)
    
    self.finalPoint = None
    self.ag = AG(20)
    self.adequacao = 0
    self.tin = 0
    

  def update(self):
    t0 = time.time()
    
    self._controller.visionSystem.update()
    
    robot = self._controller.world.robots[0]
    
    
    
    if self.finalPoint is None or (robot.x-self.finalPoint[0])**2 + (robot.y-self.finalPoint[1])**2 < 0.07**2 or time.time()-self.tin >= 5:
      
      if self._controller.world.running:
        # Fim do experimento
        #if self.tin != 0:
          #self.adequacao = self.adequacao + 50*(time.time()-self.tin)
          #print("Não adequação: " + str(self.adequacao))
          #self.ag.setAdequacao(self.adequacao)
          #self.adequacao = 0
          #self._controller.communicationSystem.sendZero()
        
        # Inicio do experimento
        self.tin = time.time()
        #individuo = self.ag.getIndividuo()
        #self._controller.controlSystem.setConstantes(individuo.ka, individuo.kg, individuo.kp)
      
      self.finalPoint = (-np.sign(robot.x)*self._controller.world.field_x_length/2*0.70, 0, -np.pi/2)
      self.trajectory = Dubins(robot.pose, self.finalPoint, 0.15)
      
      #self.trajectory = StraightLine(robot.pose, (-robot.x, robot.y, -np.pi/2))
    
#    # Integra a adequacao do experimento
#    if self._controller.world.running:
#      rmin = self.trajectory.P(self.trajectory.tmin(robot.pose))
#      p = np.sqrt((robot.x-rmin[0])**2 + (robot.y-rmin[1])**2)
#      a = rmin[2]-robot.th
#      self.adequacao = self.adequacao + (p + abs(np.sin(a))/2)
    
    # Define uma trajetória
    robot.trajectory = self.trajectory
    
    # Obtém o target instantâneo
    targets = [robot.trajectory.target(robot.pose, robot.step)]
    
    # Obtém velocidade angular e linear
    speeds = self._controller.controlSystem.actuate(targets, [])
    
    # Envia os dados via rádio
    if self._controller.world.running:
      self._controller.communicationSystem.send(speeds)
    else: self._controller.communicationSystem.sendZero()
    
    
    #print(speeds[0])
    
    #print((time.time()-t0)*1000)
    
    
