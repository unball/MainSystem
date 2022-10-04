import pickle
import socket

class ServerPickle:
    def __init__(self):

        self.host = socket.gethostname()
        self.port = 5002

        psocket = socket.socket() 
        psocket.bind((self.host, self.port))
        print('Esperando cliente.....................................')
        psocket.listen(2)
        self.conn, self.address = psocket.accept()  
        print("Connection from: " + str(self.address))
        
    def send(self, data):
        message = pickle.dumps(data,-1)
        print("Enviando: ",data)
        self.conn.send(message)
    
    def end(self):
        self.psocket.close()
