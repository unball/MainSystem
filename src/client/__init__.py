from ctypes import *
import time
from copy import copy

class VisionMessage(Structure):
    MAX_ROBOTS = 5
    _fields_ = [('n_ally', c_uint),
                ('n_enemy', c_uint),
                ('valid', c_bool),
                ('ball_x', c_double),
                ('ball_y', c_double),
                ('ally_x', c_double * MAX_ROBOTS),
                ('ally_y', c_double * MAX_ROBOTS),
                ('ally_th', c_double * MAX_ROBOTS),
                ('enemy_x', c_double * MAX_ROBOTS),
                ('enemy_y', c_double * MAX_ROBOTS),
                ('enemy_th', c_double * MAX_ROBOTS)]

class Vision:
    def __init__(self, vss, ip, port, team_yellow):
        self.lib = vss.lib
        self.lib.beginVision(c_uint(port), c_char_p(bytes(ip, 'ascii')))
        self.last_message = None
        self.team_yellow = team_yellow

    @staticmethod
    def invertMessage(message):
        invertedMessage = copy(message)
        invertedMessage['ally_x'] = copy(message["enemy_x"])
        invertedMessage['ally_y'] = copy(message["enemy_y"])
        invertedMessage['ally_th'] = copy(message["enemy_th"])
        invertedMessage['enemy_x'] = copy(message["ally_x"])
        invertedMessage['enemy_y'] = copy(message["ally_y"])
        invertedMessage['enemy_th'] = copy(message["ally_th"])
        return invertedMessage

    def read(self):
        # Define o retorno
        self.lib.visionRead.restype = VisionMessage

        # Chama a função da biblioteca compartilhada
        message = self.lib.visionRead()

        # Copia para um dicionário em python
        new_message = {
            'n_ally': message.n_ally,
            'n_enemy': message.n_enemy,
            'valid': message.valid,
            'ball_x': message.ball_x,
            'ball_y': message.ball_y,
            'ally_x': [x for x in message.ally_x],
            'ally_y': [x for x in message.ally_y],
            'ally_th': [x for x in message.ally_th],
            'enemy_x': [x for x in message.enemy_x],
            'enemy_y': [x for x in message.enemy_y],
            'enemy_th': [x for x in message.enemy_th]
        }

        # Desaloca memória da struct
        del message

        # Se a mensagem for válida retorna ela
        if new_message['valid']:
            self.last_message = new_message
            return new_message
        
        # Se não, retorna a última mensagem válida
        else: return self.last_message

class Command:
    def __init__(self, vss, ip, port, team_yellow=False):
        self.lib = vss.lib
        self.lib.beginCommand.restype = c_void_p
        self.command_p = self.lib.beginCommand(c_uint(port), c_char_p(bytes(ip, 'ascii')), c_bool(team_yellow))

    def write(self, index, vl, vr):
        self.lib.commandWrite(c_void_p(self.command_p), c_int(index), c_double(vl), c_double(vr))

    def writeMulti(self, actions):
        vls = [v[0] for v in actions]
        vrs = [v[1] for v in actions]
        self.lib.commandsWrite(c_void_p(self.command_p), (c_double * 3)(*vls), (c_double * 3)(*vrs))

    def setPos(self, index, x, y, th):
        self.lib.commandPos(c_void_p(self.command_p), c_int(index), c_double(x), c_double(y), c_double(th))

    def setBallPos(self, x, y):
        self.lib.commandBallPos(c_void_p(self.command_p), c_double(x), c_double(y))

class VSS:
    def __init__(self, hostVision="224.0.0.1", portVision=10002, hostCommand="127.0.0.1", portCommand=20011, team_yellow=False):
        self.lib = CDLL("./lib/vss.so")
        self.vision = Vision(self, hostVision, portVision, team_yellow)
        self.command = Command(self, hostCommand, portCommand, team_yellow)