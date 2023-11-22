import socketio

# Endereço do servidor SocketIO (ajuste conforme necessário)
SERVER_URL = "http://localhost:3000"

# Cria um cliente SocketIO
sio = socketio.Client()

# Conecta ao servidor
@sio.event
def connect():
    print("Conectado ao servidor")
    # Substitua 'seu_topico' e 'sua_mensagem' pelos valores desejados
    sio.emit('send_points', {'x': 1.28, 'y': -1.57})

# Desconecta do servidor
@sio.event
def disconnect():
    print("Desconectado do servidor")

# Executa a conexão
if __name__ == '__main__':
    sio.connect(SERVER_URL)
    sio.wait()

