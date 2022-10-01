import socket
import pickle

class ClientPickle:
    def __init__(self):
        self.host =socket.gethostname()
        self.port = 5001
        self.psocket = socket.socket()
        self.psocket.connect((self.host, self.port))

    def receive(self):
        data = self.psocket.recv(15000)
        message = pickle.loads(data)
        print('Received from server: ', str(message))

    def end(self):
        self.psocket.close()

if __name__ == '__main__':
    client = ClientPickle()
    while True:
        client.receive()


"""def client_program():
    host = socket.gethostname()  # as both code is running on same pc
    port = 5002  # socket server port number

    client_socket = socket.socket()  # instantiate
    client_socket.connect((host, port))  # connect to the server

    data = {
            "message_Ball": {
                "x" : 1.0,
                "y" : 2.0,
                "z" : 3.0,
                "vx" : 4.0,
                "vy" : 5.0,
                "vz" : 6.0,
            },
            "message_Robot" : {
                "robot_id" : 1,
                "x" : 2.0,
                "y" : 3.0,
                "orientation" : 4.0,
                "vx" : 5.0,
                "vy" : 6.0,
                "vorientation" : 7.0,
            }
        }
    #message = pickle.dumps(data, -1) 
    #data serialized. -1, which is an optional argument, is there to pick best the pickling protocol
    try:
        while True:
            data = input(" -> ")  # again take input
            message = pickle.dumps(data, -1) 
            client_socket.send(message)  # send message
            data = client_socket.recv(1024).decode()  # receive response
            
            print('Received from server: ' + data)  # show in terminal

            
    finally:
        # Necessita disso pois se fechar o terminal a porta vai continuar
        # reservada e aí vai precisar ficar alterando ela toda vez
        client_socket.close()  # close the connection


if __name__ == '__main__':
    client_program()"""