import sys
sys.path.append("src/")

from optimizer.optimizer import Optimizer
from optimizer.environments.FiraSIM import FiraSIM
from optimizer.systems.attacker import Attacker
from geneticalgorithm import geneticalgorithm as ga
import time
import numpy as np

class AG(Optimizer):
    def __init__(self, systemClass, environmentClass, experimentTimeout=10):
        Optimizer.__init__(self, systemClass, environmentClass)

        self.experimentTimeout = experimentTimeout
        self.experimentStart = 0

    def endExperimentCondition(self):
        return time.time() - self.experimentStart > self.experimentTimeout

    def cost(self, experimentMetrics):
        experimentMetrics = np.array(experimentMetrics)
        ballCost = 0.65 - experimentMetrics[:,0].mean()
        controlErrorCost = np.abs(experimentMetrics[:,1]).mean()
        ballDistanceCost = experimentMetrics[:,2].mean()

        return ballCost + controlErrorCost + ballDistanceCost

    def optimize(self):
        systemParams = self.systemClass.getParams()
        model = ga(function=self.execute, dimension=len(systemParams.keys()), variable_type='real', variable_boundaries=np.array(list(systemParams.values())), function_timeout=100)
        model.run()

    def execute(self, x):
        self.experimentStart = time.time()
        return self.runExperiment(x)

ag = AG(Attacker, FiraSIM)
ag.optimize()