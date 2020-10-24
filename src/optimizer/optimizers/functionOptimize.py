import sys
sys.path.append("src/")

from optimizer.optimizer import Optimizer
from optimizer.environments.FiraSIM import CornerSituation
from optimizer.systems.attacker import Attacker
from scipy.optimize import minimize
import time
import numpy as np

class FunctionOptimizer(Optimizer):
    def __init__(self, systemClass, environmentClass):
        Optimizer.__init__(self, systemClass, environmentClass)

        self.history = []

    def optimize(self):
        x0 = np.array([(v[0]+v[1])/2 for k,v in self.systemClass.getParams().items()])
        return minimize(self.execute, x0, method='Powell', tol=1e-3)

    def execute(self, x):
        cost = self.runExperiment(x)
        historyRecord = (cost, {})
        print("\n=======")
        print("cost: " + str(cost))
        for i,key in enumerate(self.systemClass.getParams()):
            historyRecord[1][key] = x[i]
            print(key + ": " + str(x[i]))
        print("=======\n")
        self.history.append(historyRecord)
        
        return cost

# class FunctionOptimizer(Optimizer):
#     def __init__(self, systemClass, environmentClass):
#         Optimizer.__init__(self, systemClass, environmentClass)

#     def optimize(self):
#         return shgo(self.execute, list(self.systemClass.getParams().values()))

#     def execute(self, x):
#         cost = self.runExperiment(x)
#         print("\n=======")
#         print("cost: " + str(cost))
#         for i,key in enumerate(self.systemClass.getParams()):
#             print(key + ": " + str(x[i]))
#         print("=======\n")
#         return cost

fo = FunctionOptimizer(Attacker, CornerSituation)
ofo = fo.optimize()