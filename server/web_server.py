from server.connect_wifi import connect_wifi
import socket
import ujson
import time
from machine import Pin
import hashlib
import binascii
import os  # Import os to check file existence 
# Configure built-in LED for connection status indication
led = Pin("LED", Pin.OUT)

# Set the path to the web folder
WEB_FOLDER = "/web"

MAGIC_STRING = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"  # WebSocket magic GUID

def start_server(ip, bot):
    try:
        # Set up socket
        addr = socket.getaddrinfo(ip, 80)[0][-1]
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(addr)
        s.listen(5)
        print("Server listening on", addr)

        while True:
            cl = None  # Initialise client socket variable
            try:
                # Accept incoming connections
                cl, client_addr = s.accept()
                print(f"Connection received from {client_addr}")

                # Read request
                request = cl.recv(2048).decode()
                if not request:
                    continue  # No request received

                print("Raw Request:", repr(request))

                # Handle request
                path = parse_request_path(request)
                if path and not path.startswith("/ws"):
                    serve_file(cl, path)
                elif request.startswith("GET /ws") and "Upgrade: websocket" in request:
                    print("WebSocket upgrade request detected")
                    ws = websocket_handshake(cl)
                    if ws:
                        print("WebSocket handshake completed")
                        handle_websocket(ws, bot)
                    else:
                        print("WebSocket handshake failed")
            except Exception as e:
                print("Error handling client request:", e)
            finally:
                if cl:
                    cl.close()  # Ensure the client socket is always closed

    except Exception as e:
        print("Server failed to start:", e)
    finally:
        if s:
            s.close()  # Ensure the server socket is always closed


def parse_request_path(request):
    """Extracts the requested file path from the HTTP GET request."""
    lines = request.split("\r\n")
    if lines:
        first_line = lines[0]
        parts = first_line.split(" ")
        if len(parts) > 1 and parts[0] == "GET":
            return parts[1] if parts[1] != "/" else "/index.html"
    return None

def serve_file(cl, path):
    """Serves a static file from the web folder."""
    file_path = f"{WEB_FOLDER}{path}"
    try:
        if os.stat(file_path):  # Check if the file exists
            with open(file_path, "r") as f:
                response = f"HTTP/1.0 200 OK\r\nContent-Type: {get_content_type(path)}\r\n\r\n"
                cl.send(response)
                for line in f:
                    cl.send(line)
    except OSError:
        # File not found
        cl.send("HTTP/1.0 404 Not Found\r\n\r\nFile Not Found")

def get_content_type(path):
    """Determines the MIME type based on the file extension."""
    if path.endswith(".html"):
        return "text/html"
    elif path.endswith(".css"):
        return "text/css"
    elif path.endswith(".js"):
        return "application/javascript"
    else:
        return "text/plain"

def websocket_handshake(cl):
    try:
        # Read full HTTP request
        cl.settimeout(5)  # Set a timeout to avoid indefinite hangs
        request = ""
        while True:
            part = cl.recv(1024).decode()
            request += part
            if "\r\n\r\n" in request:  # HTTP header terminator
                break
        print("Full Handshake Request:", repr(request))


        # Extract Sec-WebSocket-Key
        key = None
        for line in request.split("\r\n"):
            if line.lower().startswith("sec-websocket-key"):
                key = line.split(":")[1].strip()
                break

        if not key:
            print("WebSocket error: No Sec-WebSocket-Key found in request")
            cl.send("HTTP/1.1 400 Bad Request\r\n\r\n")
            return None

        # Generate Sec-WebSocket-Accept
        accept_key = hashlib.sha1((key + MAGIC_STRING).encode()).digest()
        accept_key = binascii.b2a_base64(accept_key).decode().strip()

        # Construct the handshake response
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_key}\r\n\r\n"
        )
        cl.send(response.encode())  # Send handshake response
        print("WebSocket handshake successful")
        return cl  # Return the WebSocket connection

    except Exception as e:
        print("WebSocket handshake failed:", e)
        return None


def handle_websocket(ws, bot):
    if ws is None:
        print("WebSocket connection not established; exiting handler.")
        return

    print("WebSocket connection established")
    last_ping_time = time.time()

    while True:
        try:
            # Check for periodic ping
            current_time = time.time()
            if current_time - last_ping_time > 10:  # Send a ping every 10 seconds
                ws.send(encode_websocket_frame("ping"))
                last_ping_time = current_time

            # Non-blocking receive with timeout
            ws.settimeout(1)  # Check every second for new data
            try:
                data = ws.recv(2048)
                if data:
                    payload = decode_websocket_frame(data)
                    if payload:
                        handle_control(payload, bot)
            except OSError:
                # Ignore timeout errors
                pass

        except Exception as e:
            print("WebSocket error:", e)
            break

def encode_websocket_frame(message):
    message_bytes = message.encode("utf-8")
    frame = bytearray()
    frame.append(0x81)  # FIN + text frame
    length = len(message_bytes)
    if length <= 125:
        frame.append(length)
    elif length <= 65535:
        frame.append(126)
        frame.extend(length.to_bytes(2, 'big'))
    else:
        frame.append(127)
        frame.extend(length.to_bytes(8, 'big'))
    frame.extend(message_bytes)
    return frame



def decode_websocket_frame(data):
    if len(data) < 6:
        print("WebSocket frame error: Incomplete frame received")
        return None

    length = data[1] & 127
    if length == 126:
        mask = data[4:8]
        decoded = bytearray(data[8:])
    elif length == 127:
        mask = data[10:14]
        decoded = bytearray(data[14:])
    else:
        mask = data[2:6]
        decoded = bytearray(data[6:])

    for i in range(len(decoded)):
        decoded[i] ^= mask[i % 4]

    try:
        decoded_message = decoded.decode("utf-8")
        print("Decoded WebSocket message:", decoded_message)
        return decoded_message
    except UnicodeDecodeError as e:
        print("WebSocket decode error:", e)
        return None

def handle_control(payload, bot):
    try:
        print("Received payload:", payload)
        data = ujson.loads(payload)
        
        Vx = data.get("Vx", 0)
        Vy = data.get("Vy", 0)
        omega = data.get("omega", 0)
        
        print("Parsed Data - Vx:", Vx, "Vy:", Vy, "omega:", omega)
        bot.set_velocity(Vx=Vx, Vy=Vy, omega=omega)
        
    except Exception as e:
        print("Failed to handle control:", e)
        

if __name__ == "__main__":
    # Connect to WiFi
    ip = connect_wifi()
    print("Starting server...")

    # Define a mock bot class for testing
    class MockBot:
        def set_velocity(self, Vx, Vy, omega):
            print(f"Setting velocity: Vx={Vx}, Vy={Vy}, omega={omega}")

    # Create a mock bot instance for testing
    bot = MockBot()
    start_server(ip,bot)
