import numpy as np

def norm(p0: tuple, p1: tuple):
  return np.sqrt((p0[0]-p1[0])**2+(p0[1]-p1[1])**2)
