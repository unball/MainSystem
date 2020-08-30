from ctypes import *
import time

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
    def __init__(self, vss, ip, port):
        self.lib = vss.lib
        self.lib.beginVision(c_uint(port), c_char_p(bytes(ip, 'ascii')))
        self.last_message = None

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
    def __init__(self, vss, ip, port):
        self.lib = vss.lib
        self.lib.beginCommand(c_uint(port), c_char_p(bytes(ip, 'ascii')))

    def write(self, index, vl, vr):
        self.lib.commandWrite(c_int(index), c_double(vl), c_double(vr))

class VSS:
    def __init__(self):
        self.lib = CDLL("./lib/vss.so")
        self.vision = Vision(self, "127.0.0.1", 10020)
        self.command = Command(self, "127.0.0.1", 20011)