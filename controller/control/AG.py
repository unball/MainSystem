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
