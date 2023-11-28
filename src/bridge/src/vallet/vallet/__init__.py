import os
import vallet
import socketio

socket_client = socketio.Client()
socket_client.connect(os.environ.get("SOCKET_URL"))
