import socketio
import time
import threading  # Importa o módulo threading para controle de execução

# Endereço do servidor SocketIO (ajuste conforme necessário)
SERVER_URL = "http://localhost:3000"

# Cria um cliente SocketIO
sio = socketio.Client()

sio.connect(SERVER_URL)

# Variável de controle para interromper a execução
running = True

cords = {
    "beats": {'x': 1.44, 'y': -0.1},
    "brahma": {'x': 1.42, 'y': -1.53},
    "skol": {'x': 0.17, 'y': -1.47},
    "almoxarifado": {'x': 0.0, 'y': 0.0},
    "ze": {'x': 0.75, 'y': -0.82},
}

# Função para enviar pontos a cada dois segundos
def send_points():
    while running:
        time.sleep(2)
        for cord in cords.values():
            sio.emit('enqueue', cord)
            print("Enviado: ", cord)

# Conecta ao servidor
@sio.event
def connect():
    print("Conectado ao servidor")
    send_points()

# Desconecta do servidor
@sio.event
def disconnect():
    global running  # Acessa a variável global para interromper a execução
    running = False
    print("Desconectado do servidor")

# Executa a conexão
if __name__ == '__main__':
    try:
        # Inicia uma thread para enviar pontos
        t = threading.Thread(target=send_points)
        t.start()

        # Aguarda a thread de envio de pontos encerrar
        t.join()

    except KeyboardInterrupt:
        # Permite encerrar o script com Ctrl+C
        pass
    finally:
        # Desconecta do servidor ao finalizar
        sio.disconnect()
