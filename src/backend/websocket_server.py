import socketio
import eventlet

# Criando um servidor SocketIO
sio = socketio.Server(cors_allowed_origins='*')

# Evento de conexão
@sio.event
def connect(sid, environ):
    print('Cliente conectado:', sid)

# Evento para lidar com mensagens recebidas
@sio.on('enqueue')
def message(sid, data):
    print(f'Mensagem recebida de {sid}: {data}')
    sio.emit("/enqueue", data)

@sio.on('/battery')
def message(sid, data):
    print(f'Mensagem recebida de {sid}: {data}')
    sio.emit("message", data)

# Evento de desconexão
@sio.event
def disconnect(sid):
    print('Cliente desconectado:', sid)

# Embrulhando com WSGI middleware
app = socketio.WSGIApp(sio)

# Rodar o servidor
if __name__ == '__main__':
    print('Iniciando o servidor SocketIO...')
    eventlet.wsgi.server(eventlet.listen(('', 3000)), app)
