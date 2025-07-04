import network
import socket
import ujson
import os
import time
from machine import reset, Pin

class WiFiManager:
    def __init__(self, config_file="wifi_config.json", web_root="/web", led_pin=25):
        self.config_file = config_file
        self.web_root = web_root
        self.sta_if = network.WLAN(network.STA_IF)
        self.ap_if = network.WLAN(network.AP_IF)
        self.led = Pin("LED", Pin.OUT)

    def auto_connect(self):
        print("Attempting auto-connect...")
        self._led_flash_slow()
        connected = self.connect_to_known()
        if connected:
            self._led_on()
            print("Connected:", self.sta_if.ifconfig())
            return self.sta_if.ifconfig()[0]  # Return IP address
        else:
            print("Failed to connect. Starting AP mode.")
            self._led_flash_fast()
            self.start_access_point()
            self.start_config_server()  # <== Blocking call that reboots on save
            return None  # Not strictly necessary since reboot occurs

    # === CONFIG FILE HANDLING ===

    def load_config(self):
        try:
            with open(self.config_file, "r") as f:
                return ujson.load(f)
        except:
            return {}

    def save_config(self, ssid, password):
        configs = self.load_config()
        configs[ssid] = password
        with open(self.config_file, "w") as f:
            ujson.dump(configs, f)

    def delete_config(self, ssid):
        configs = self.load_config()
        if ssid in configs:
            del configs[ssid]
            with open(self.config_file, "w") as f:
                ujson.dump(configs, f)

    def get_all_configs(self):
        return self.load_config()

    def edit_config(self, ssid, new_password):
        configs = self.load_config()
        configs[ssid] = new_password
        with open(self.config_file, "w") as f:
            ujson.dump(configs, f)

    # === NETWORK FUNCTIONS ===

    def scan_networks(self):
        self.sta_if.active(True)
        return [net[0].decode('utf-8') for net in self.sta_if.scan()]

    def connect_to_known(self, timeout=15):
        configs = self.load_config()
        if not configs:
            return False

        self.sta_if.active(True)
        available = self.scan_networks()

        for ssid in available:
            if ssid in configs:
                print(f"Trying to connect to {ssid}...")
                self.sta_if.connect(ssid, configs[ssid])
                for _ in range(timeout * 10):
                    if self.sta_if.isconnected():
                        print("Connected.")
                        return True
                    self._led_toggle()
                    time.sleep(0.5)
                print(f"Failed to connect to {ssid}.")
        return False

    def start_access_point(self, ssid="PicoSetup", password="12345678"):
        self.ap_if.config(essid=ssid, password=password)
        self.ap_if.active(True)
        print(f"Access point '{ssid}' started.")

    def stop_access_point(self):
        self.ap_if.active(False)

    # === CONFIGURATION WEB SERVER ===

    def start_config_server(self, port=80):
        addr = socket.getaddrinfo('0.0.0.0', port)[0][-1]
        s = socket.socket()
        s.bind(addr)
        s.listen(1)
        print("Config server running on http://192.168.4.1")

        while True:
            cl, addr = s.accept()
            try:
                request = cl.recv(1024).decode()
                path = self._get_path(request)
                print("Request for:", path)

                if "POST /save" in request:
                    params = self._parse_post_data(request)
                    self.save_config(params.get("ssid", ""), params.get("password", ""))
                    self._send_html(cl, "<h3>Saved. Rebooting...</h3>")
                    cl.close()
                    time.sleep(2)
                    reset()

                elif "POST /edit" in request:
                    params = self._parse_post_data(request)
                    self.edit_config(params.get("ssid", ""), params.get("password", ""))
                    self._send_html(cl, "<h3>Updated. Rebooting...</h3>")
                    cl.close()
                    time.sleep(2)
                    reset()

                elif "POST /delete" in request:
                    params = self._parse_post_data(request)
                    self.delete_config(params.get("ssid", ""))
                    self._send_html(cl, "<h3>Deleted. Rebooting...</h3>")
                    cl.close()
                    time.sleep(2)
                    reset()

                elif path == "/" or path == "/index.html":
                    self._serve_file(cl, "/wificonfig.html")

                elif path == "/style.css":
                    self._serve_file(cl, "/style.css", content_type="text/css")

                elif path == "/networks.json":
                    networks = self.scan_networks()
                    self._send_json(cl, networks)

                elif path == "/configs.json":
                    configs = list(self.get_all_configs().keys())
                    self._send_json(cl, configs)

                else:
                    self._send_html(cl, "<h3>404 Not Found</h3>", status="404 Not Found")

            except Exception as e:
                print("Error:", e)
            finally:
                cl.close()

    # === LED METHODS ===

    def _led_on(self):
        self.led.value(1)

    def _led_off(self):
        self.led.value(0)

    def _led_toggle(self):
        self.led.toggle()

    def _led_flash_slow(self):
        self._led_off()
        for _ in range(6):  # ~3 seconds
            self._led_toggle()
            time.sleep(0.5)

    def _led_flash_fast(self):
        self._led_off()
        for _ in range(10):  # ~1 second
            self._led_toggle()
            time.sleep(0.1)

    # === UTILITIES ===

    def _get_path(self, request):
        try:
            first_line = request.split("\r\n")[0]
            parts = first_line.split(" ")
            if len(parts) > 1:
                return parts[1]
        except:
            pass
        return "/"

    def _serve_file(self, cl, filename, content_type="text/html"):
        path = self.web_root + filename
        try:
            with open(path, "r") as f:
                content = f.read()
            cl.send(f"HTTP/1.1 200 OK\r\nContent-Type: {content_type}\r\n\r\n")
            cl.send(content)
        except OSError:
            cl.send("HTTP/1.1 404 Not Found\r\n\r\nFile not found")

    def _send_html(self, cl, html, status="200 OK"):
        cl.send(f"HTTP/1.1 {status}\r\nContent-Type: text/html\r\n\r\n")
        cl.send(f"<html><body>{html}</body></html>")

    def _send_json(self, cl, data):
        cl.send("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n")
        cl.send(ujson.dumps(data))

    def _parse_post_data(self, request):
        try:
            body = request.split("\r\n\r\n", 1)[1]
            pairs = body.split("&")
            data = {}
            for pair in pairs:
                if "=" in pair:
                    key, value = pair.split("=", 1)
                    data[key] = value.replace("+", " ").replace("%3A", ":").replace("%2F", "/")
            return data
        except:
            return {}
