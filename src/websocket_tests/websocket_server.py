import socketio
import eventlet
from dotenv import load_dotenv
from utils.manage_pool import DB_Pool
from utils.order_table import Order

load_dotenv()

db_pool = None

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
        sio.emit("add_to_queue", data)

@sio.on('robot_status')
def message(sid, data):
    print(f'Mensagem recebida de {sid}: {data}')
    sio.emit("add_to_queue", data)

@sio.on("put_on_db")
def put_on_db(sid, data):
    print(f'Mensagem recebida de {sid}: {data}')
    order = Order()
    temp_conn = db_pool.get_connection()
    try:
        order.create(temp_conn, data)
    except:
        print("Error on creating order")
    finally:
        db_pool.put_connection(temp_conn)

# Evento de desconexão
@sio.event
def disconnect(sid):
    print('Cliente desconectado:', sid)

# Embrulhando com WSGI middleware
app = socketio.WSGIApp(sio)

# Rodar o servidor
if __name__ == '__main__':
    db_pool = DB_Pool()
    print('Iniciando o servidor SocketIO...')
    eventlet.wsgi.server(eventlet.listen(('', 3000)), app)
