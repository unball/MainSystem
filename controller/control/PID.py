from model.paramsPattern import ParamsPattern

class PID(ParamsPattern):
  def __init__(self, name):
    ParamsPattern.__init__(self, name, {"kp": 1, "ki": 1, "kd": 1, "sat": 1, "enable": False})
    self.integrated = 0
    self.lastError = 0
    
  def actuate(self, reference, current, dt):
    if not self.getParam("enable"): return reference
    
    error = reference-current
    derivated = error-self.lastError
    
    self.integrated = max(min(self.integrated+error*dt, self.getParam("sat")), -self.getParam("sat"))
    self.lastError = error
    
    return self.getParam("kp") * error + self.getParam("ki") * self.integrated + self.getParam("kd") * derivated
