import socketio
import eventlet
import json

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
    
    # Verificar se o tipo do dado recebido é suportado
    if isinstance(data, (str, dict)):
        try:
            if isinstance(data, str):
                try:
                    dada = json.loads(data)
                except json.JSONDecodeError:
                    pass
            json_data = json.dumps(data)
            sio.emit("/enqueue", json_data)
        except TypeError as e:
            print(f"Error ao serializar: {e}")

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
