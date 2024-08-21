import socketio # type: ignore

class Socket:
    def __init__(self, server_url):
        # Create a Socket.IO client with automatic reconnection
        self.sio = socketio.Client(reconnection=True, reconnection_attempts=float('inf'), reconnection_delay=1)
        self.server_url = server_url

        # Register event handlers
        self.register_events()
        self.isConnect = False
        
    def register_events(self):
        @self.sio.event
        def connect():
            self.isConnect = True
            print(f"Connected to server, {self.isConnect}")

        @self.sio.event
        def disconnect():
            self.isConnect = False
            print(f"Disconnected from server, {self.isConnect}")

        @self.sio.event
        def connect_error(data):
            print(f"Failed to connect to server: {data}")

    def connect(self):
        self.sio.connect(self.server_url)

    def disconnect(self):
        self.sio.disconnect()
    
    def send_message(self, stream, direction, header, data, status = None):
        # print(self.sio.connected)
        if self.sio.connected:
            self.sio.emit(stream, {
                "direction": direction,
                "header": header,
                "data": data,
                "status": status
            })
        else:
            print("Socket.IO client is not connected. Cannot send message.")

def handle_drone_event(data):
    print(f"Drone event received: {data}")

if __name__ == "__main__":
    
    server_url = 'http://103.167.198.50:5000'
    socket_client = Socket(server_url)
    
    socket_client.register_events()
    socket_client.sio.on('drone', handle_drone_event)
    
    try:
        # Connect to the server
        socket_client.connect()
        
        # Send a message to the server
        # socket_client.send_message('Hello, server!')

        socket_client.send_message("controlMsg", "web", "droneStatus", "Hello, server!")
        
        # To keep the client running and receiving messages
        try:
            while True:
                pass
        except KeyboardInterrupt:
            # Disconnect from the server on exit
            socket_client.disconnect()
        
    except Exception as e:
        print("error in duration connect to server")
        
    
