import socketio
import eventlet

# Criando um servidor SocketIO
sio = socketio.Server(cors_allowed_origins='*')

# Evento de conexão
@sio.event
def connect(sid, environ):
    print('Cliente conectado:', sid)

# Evento para lidar com mensagens recebidas
@sio.on('send_points')
def message(sid, data):
    if data['x'] and data['y']:
        print(f'Mensagem recebida de {sid}: {data}')
        sio.emit("/navigation", data)

@sio.on('robot_status')
def message(sid, data):
    print(f'Mensagem recebida de {sid}: {data}')
    sio.emit("/navigation", data)


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
