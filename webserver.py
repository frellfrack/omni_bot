# webserver.py

import uasyncio as asyncio
import ujson
import ubinascii
import uhashlib
import os

class WebServer:
    def __init__(self, router, static_dir="web", port=80, on_connect=None, on_disconnect=None, on_error=None):
        self.router = router
        self.static_dir = static_dir
        self.port = port
        self.on_connect = on_connect
        self.on_disconnect = on_disconnect
        self.on_error = on_error

    def _websocket_accept(self, key):
        GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        hash = uhashlib.sha1((key + GUID).encode()).digest()
        return ubinascii.b2a_base64(hash).strip().decode()

    def _encode_ws(self, msg):
        encoded = msg.encode()
        length = len(encoded)
        if length < 126:
            return bytes([0x81, length]) + encoded
        elif length < (1 << 16):
            return bytes([0x81, 126]) + length.to_bytes(2, 'big') + encoded
        else:
            return bytes([0x81, 127]) + length.to_bytes(8, 'big') + encoded

    async def _serve_file(self, writer, path):
        if path == "/":
            path = "/index.html"
        full_path = self.static_dir + path
        try:
            ext = path.split(".")[-1]
            types = {
                "html": "text/html",
                "js": "application/javascript",
                "css": "text/css",
                "svg": "image/svg+xml",
            }
            mime = types.get(ext, "text/plain")
            await writer.awrite(f"HTTP/1.0 200 OK\r\nContent-Type: {mime}\r\n\r\n")
            with open(full_path, "r") as f:
                while True:
                    data = f.read(512)
                    if not data:
                        break
                    await writer.awrite(data)
        except:
            await writer.awrite("HTTP/1.0 404 Not Found\r\n\r\n404 Not Found")

    async def _handle_ws(self, reader, writer):
        try:
            while True:
                header = await reader.readexactly(2)
                if not header or len(header) < 2:
                    break

                fin_and_opcode = header[0]
                masked_and_length = header[1]
                payload_len = masked_and_length & 0x7F

                if payload_len == 126:
                    extended = await reader.readexactly(2)
                    payload_len = int.from_bytes(extended, 'big')
                elif payload_len == 127:
                    extended = await reader.readexactly(8)
                    payload_len = int.from_bytes(extended, 'big')

                mask = await reader.readexactly(4)
                payload = await reader.readexactly(payload_len)

                decoded = ''.join(chr(payload[i] ^ mask[i % 4]) for i in range(payload_len))

                try:
                    msg = ujson.loads(decoded)
                    response = self.router.handle(msg)
                    if response:
                        await writer.awrite(self._encode_ws(response))
                except Exception as e:
                    print("WebSocket decode error:", e)

        except Exception as e:
            print("WebSocket connection error:", e)

    async def _handle_client(self, reader, writer):
        try:
            req = await reader.readline()
            headers = {}
            while True:
                line = await reader.readline()
                if line == b"\r\n":
                    break
                parts = line.decode().split(":", 1)
                if len(parts) == 2:
                    headers[parts[0].strip()] = parts[1].strip()

            if "Upgrade" in headers and headers["Upgrade"].lower() == "websocket":
                key = headers["Sec-WebSocket-Key"]
                accept = self._websocket_accept(key)
                response = (
                    "HTTP/1.1 101 Switching Protocols\r\n"
                    "Upgrade: websocket\r\n"
                    "Connection: Upgrade\r\n"
                    f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
                )
                await writer.awrite(response)
                if self.on_connect:
                    self.on_connect()    
                await self._handle_ws(reader, writer)
            else:
                path = req.decode().split()[1]
                await self._serve_file(writer, path)
        except Exception as e:
            if self.on_error:
                self.on_error(e)
            print("Client error:", e)
        finally:
            await writer.aclose()
            if self.on_disconnect:
                self.on_disconnect()
            print("Client disconnected")

    async def start(self):
        await asyncio.start_server(self._handle_client, "0.0.0.0", self.port)
        print("HTTP/WebSocket server ready")
