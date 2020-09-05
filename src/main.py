from client.client import VSS
import time

vss = VSS()

t0 = 0

def loop():
    global t0
    # Lê a visão
    message = vss.vision.read()

    # Visão é inválida
    if message is None: return

    # Mostra os dados da visão
    print("       \t  x  \t  y  \t  th  ")
    print("ball\t{:+2.2f}\t{:+2.2f}".format(message['ball_x'], message['ball_y']))
    for i in range(message['n_ally']):
        x,y,th = message['ally_x'][i], message['ally_y'][i], message['ally_th'][i]
        print("ally {:d}\t{:+2.2f}\t{:+2.2f}\t{:+2.2f}".format(i,x,y,th))
    for i in range(message['n_enemy']):
        x,y,th = message['enemy_x'][i], message['enemy_y'][i], message['enemy_th'][i]
        print("enemy {:d}\t{:+2.2f}\t{:+2.2f}\t{:+2.2f}".format(i,x,y,th))

    # Controla
    for i in range(message['n_ally']):
        if abs(message['ally_x'][i]) < 0.1:
            vss.command.setPos(i, -0.6, 0, 0)
        elif message['ally_x'][i] <= 0:
            vss.command.write(i, 10, 10)
        else:
            vss.command.write(i, -10, -10)

    # Atualiza o tempo
    tn = time.time()
    print("{:.2f}".format(1/(tn-t0)) + "Hz")
    t0 = tn

    # Espera
    time.sleep(1.0 / 120)

while True:
    loop()