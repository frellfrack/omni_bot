import socket
import json
import time

# WebSocket Server
def start_websocket_server(bot, ip_address, port=8080):
    # Create a socket and bind it to the specific IP and port
    addr = (ip_address, port)
    s = socket.socket()
    s.bind(addr)
    s.listen(1)
    print(f"WebSocket server listening on {ip_address}:{port}")

    while True:
        # Accept a client connection
        cl, addr = s.accept()
        print('Client connected from', addr)
        
        # Perform WebSocket handshake
        request = cl.recv(1024)
        if b"Upgrade: websocket" in request:
            # Respond with WebSocket handshake headers
            cl.send(b'HTTP/1.1 101 Switching Protocols\r\n'
                    b'Upgrade: websocket\r\n'
                    b'Connection: Upgrade\r\n'
                    b'Sec-WebSocket-Accept: ' + b'\r\n\r\n')
            print("WebSocket handshake complete")

            # Connection loop to handle commands
            connected = True
            while connected:
                try:
                    # Receive and decode the message
                    data = cl.recv(1024)
                    if not data:
                        connected = False
                        break

                    # Parse the JSON command
                    message = data.decode("utf-8").strip()
                    command = json.loads(message)  # Expecting JSON like {"Vx": 0.5, "Vy": 0.2, "omega": 0.1}
                    vx = command.get("Vx", 0.0)
                    vy = command.get("Vy", 0.0)
                    omega = command.get("omega", 0.0)

                    # Check if command is (0, 0, 0) and stop if so
                    if vx == 0 and vy == 0 and omega == 0:
                        bot.stop()
                        print("Bot stopped")
                    else:                       
                        #Calculate target velocities for each wheel
                        front_left_v,rear_left_v,front_right_v,rear_right_v = bot.calculate_velocities(Vx, Vy, omega)
                              
                        #Move the bot with specified velocities for a set duration.
                        bot.set_velocity(front_left_v,rear_left_v,front_right_v,rear_right_v)
                        # Set bot velocity and update motors
                        bot.set_velocity(vx, vy, omega)
                        bot.update_motors()  # Continuously update motors

                    # Optionally, send a confirmation back to the client
                    cl.send(b"Command received and executed")

                    # Add a short pause between updates
                    time.sleep(0.1)

                except Exception as e:
                    print("Connection error:", e)
                    connected = False
                    break
            cl.close()
            print("Client disconnected")
