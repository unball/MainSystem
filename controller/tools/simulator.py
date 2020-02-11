from controller.tools import adjustAngle, ang
import numpy as np

def simulate(robot, v, w, dt=0.033, r=0.03, L=0.075):
    """Esta função implementa um simulador para o robô"""
    if w != 0:
        R = abs(v) / w
        rc = (robot.x - R * np.sin(robot.th), robot.y + R * np.cos(robot.th))
        dth = w * dt
        x = rc[0] + abs(R) * np.cos(ang(rc, robot.pos) + dth)
        y = rc[1] + abs(R) * np.sin(ang(rc, robot.pos) + dth)
        th = adjustAngle(robot.th + dth)
    else:
        x = robot.x + v * dt * np.cos(robot.th)
        y = robot.y + v * dt * np.sin(robot.th)
        th = robot.th

    robot.update(x,y,th,rawUpdate=False)

t = 0
m = 1
def simulateBall(ball, dt=0.033):
    global t
    global m
    if abs(t) >= 1: m *= -1
    t = t + dt * m
    x = 0.4 * t
    y = 0.4 * t
    ball.update(x,y,0)
    ball.calc_velocities(dt)