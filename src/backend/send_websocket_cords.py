import socketio
import time
import threading  # Importa o módulo threading para controle de execução

# Endereço do servidor SocketIO (ajuste conforme necessário)
SERVER_URL = "http://10.128.65.252:3030"

# Cria um cliente SocketIO
sio = socketio.Client()

sio.connect(SERVER_URL)

# Variável de controle para interromper a execução
running = True

# emergency = {
#     "beats": {"emergency_stop": 0}
#     }

# # Função para enviar pontos a cada dois segundos    
# def send_points():
#     while running:
#         time.sleep(2)
#         for label in emergency.values():
#             sio.emit('emergency_stop', label)
#             print("Enviado: ", label)

cords = {
    "beats": {"id":'1', "type": "GRAB", "x": 2.5, "y": 2.5},
    "brama": {"id":'2', "type": "DROP","x": 0.0, "y": 0.0},
    }

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
