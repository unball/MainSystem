import sys
sys.path.append("src/")

from optimizer.optimizer import Optimizer
from optimizer.environments.FiraSIM import CornerSituation
from optimizer.systems.attacker import Attacker
from geneticalgorithm import geneticalgorithm as ga
import time
import numpy as np

class AG(Optimizer):
    def __init__(self, systemClass, environmentClass):
        Optimizer.__init__(self, systemClass, environmentClass)

    def optimize(self):
        systemParams = self.systemClass.getParams()
        model = ga(function=self.execute, dimension=len(systemParams.keys()), variable_type='real', variable_boundaries=np.array(list(systemParams.values())), function_timeout=100)
        model.run()

    def execute(self, x):
        cost = self.runExperiment(x)
        print("\n=======")
        print("cost: " + str(cost))
        for i,key in enumerate(self.systemClass.getParams()):
            print(key + ": " + str(x[i]))
        print("=======\n")
        return 1000-cost

ag = AG(Attacker, CornerSituation)
ag.optimize()