import numpy as np

class AG():
  def __init__(self, popsize, numgens, low, high):
    self.pop = np.random.uniform(size=(popsize, numgens))
    self.pop = self.pop*(high-low) + low
    self.fit = np.zeros(popsize)
    self.chromosome = 0
  
  def updateCurrentFit(self, fit):
    self.fit[self.chromosome] = fit
  
  def experimentOver(self):
    return self.chromosome >= self.pop.size
    
  def selection(self, accProb, size):
    return np.array([np.argwhere(accProb >= np.random.uniform())[0,0] for i in range(size)])
  
  def reproduce(self):
    accProb = np.add.accumulate(self.fit / self.fit.sum())
    
    popsize, numgens = self.pop.shape
    halfpop = popsize // 2
    fathers = self.selection(accProb, halfpop)
    mothers = self.selection(accProb, halfpop)
    

#class Individuo():
#  def __init__(self, params):
#    self.params = params
#    self.adequacao = 0
#    
#  @property
#  def ka(self):
#    return self.params[0]
#    
#  @property
#  def kg(self):
#    return self.params[1]
#    
#  @property
#  def kp(self):
#    return self.params[2]

#class AG():
#  def __init__(self, size):
#    self.currentIndividuo = 0
#    
#    self.weights = [10,500,5]
#    self.min = [4, 0, 7]
#    
#    self.initialGen = [{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
#{"ka": 1.1172688438443967, "kg": 109.4501515549437, "kp": 5.9396152856884},
#{"ka": 6.699208305468581, "kg": 382.8649280089019, "kp": 2.377935126960727},
#{"ka": 5.837680521958922, "kg": 133.61676185098975, "kp": 4.555786188293416},
#{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
#{"ka": 5.837680521958922, "kg": 133.61676185098975, "kp": 4.555786188293416},
#{"ka": 5.617773757461554, "kg": 14.683103375551232, "kp": 5.9396152856884},
#{"ka": 5.617773757461554, "kg": 14.683103375551232, "kp": 5.9396152856884},
#{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
#{"ka": 5.617773757461554, "kg": 14.683103375551232, "kp": 5.9396152856884},
#{"ka": 5.837680521958922, "kg": 14.683103375551232, "kp": 5.9396152856884},
#{"ka": 4.7270998549824705, "kg": 118.99659632111248, "kp": 5.870844036094553},
#{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
#{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
#{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
#{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 2.5297938123647503},
#{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553},
#{"ka": 5.837680521958922, "kg": 133.61676185098975, "kp": 4.555786188293416},
#{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 9.385447667773759},
#{"ka": 5.837680521958922, "kg": 118.99659632111248, "kp": 5.870844036094553}]
#    
#    self.individuos = [Individuo([self.weights[0]*np.random.rand()+self.min[0], self.weights[1]*np.random.rand()+self.min[1], self.weights[2]*np.random.rand() + self.min[2]]) for i in range(size)]
#    #self.individuos = [Individuo([self.initialGen[i]["ka"], self.initialGen[i]["kg"], self.initialGen[i]["kp"]]) for i in range(size)]
#  
#    
#  def setAdequacao(self, valor):
#    self.individuos[self.currentIndividuo].adequacao = valor
#    self.currentIndividuo = self.currentIndividuo + 1
#    
#    # Fim do experimento com todos os indivíduos
#    if self.currentIndividuo == len(self.individuos):
#      self.reproduzir()
#      self.currentIndividuo = 0
#      
#  def getIndividuo(self):
#    return self.individuos[self.currentIndividuo]
#    
#  def corrigirAdequacao(self):
#    maiorAdequacao = max(self.individuos, key=lambda x: x.adequacao).adequacao
#    soma = 0
#    for i in range(len(self.individuos)):
#      self.individuos[i].adequacao = maiorAdequacao-self.individuos[i].adequacao
#      soma = soma + self.individuos[i].adequacao
#    return soma
#    
#    
#  def encontrarIndividuo(self, regioes, prob):
#    for i in range(len(self.individuos)):
#      if regioes[i][0] <= prob and regioes[i][1] >= prob:
#        return i
#    return None
#      
#  def reproduzir(self):
#    somaAdequacao = self.corrigirAdequacao()
#    
#    print("Resultado indivíduos:")
#    for individuo in sorted(self.individuos, key=lambda x:-x.adequacao):
#      print("Adequação: {3}\tka: {0}\tkg: {1}\tkp: {2}".format(individuo.ka, individuo.kg, individuo.kp, individuo.adequacao))
#      
#    listaRegiaoProb = []
#    probAnt = 0
#    for i in range(len(self.individuos)):
#      prob = self.individuos[i].adequacao/somaAdequacao
#      listaRegiaoProb.append((probAnt,probAnt+prob))
#      probAnt = probAnt+prob
#      
#    print(listaRegiaoProb)
#    
#    pais = []
#    maes = []
#    for i in range(len(self.individuos)//2):
#      pais.append(self.individuos[self.encontrarIndividuo(listaRegiaoProb, np.random.rand())])
#      maes.append(self.individuos[self.encontrarIndividuo(listaRegiaoProb, np.random.rand())])
#    
#    novosIndividuos = []
#    for i in range(len(self.individuos)//2):
#      pc = np.random.rand()
#      if pc > 0.7:
#        locus = np.random.randint(3)+1
#        novosIndividuos.append(Individuo(pais[i].params[:locus]+maes[i].params[locus:3]))
#        novosIndividuos.append(Individuo(maes[i].params[:locus]+pais[i].params[locus:3]))
#      else:
#        novosIndividuos.append(Individuo(pais[i].params))
#        novosIndividuos.append(Individuo(maes[i].params))
#      
#      for j in range(len(novosIndividuos[-1].params)):
#        pm1 = np.random.rand()
#        pm2 = np.random.rand()
#        if pm1 > 0.97:
#          novosIndividuos[-1].params[j] = self.weights[j] * np.random.rand() + self.min[j]
#        if pm2 > 0.97:
#          novosIndividuos[-2].params[j] = self.weights[j] * np.random.rand() + self.min[j]
#    
#    print("Novos indivíduos:")
#    for individuo in novosIndividuos:
#      print("ka: {0}\tkg: {1}\tkp: {2}".format(individuo.ka, individuo.kg, individuo.kp))
#    
#    self.individuos = novosIndividuos

