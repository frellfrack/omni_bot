import network
import time
from config import WIFI_SSID, WIFI_PASSWORD, NETWORK_NAME
from machine import Pin

try:
    import mdns  # Import mDNS library if available
except ImportError:
    mdns = None
    
led = Pin("LED", Pin.OUT)

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)  # Ensure there's no call syntax error here
    wlan.active(True)

    try:
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        
        # Flash LED while attempting to connect
        attempts = 0
        while not wlan.isconnected():
            led.toggle()
            print("Connecting to WiFi...")
            time.sleep(0.5)
            attempts += 1
            
            # Optional: Limit the number of connection attempts
            if attempts > 30:  # Roughly 15 seconds
                raise RuntimeError("WiFi connection timed out.")
        
        # LED solid on for 2 seconds upon successful connection, then off
        led.on()
        ip_address = wlan.ifconfig()[0]
        print("Connected to WiFi. IP address:", ip_address)
        #network.hostname(NETWORK_NAME)
        time.sleep(2)
        led.on()
         
        return ip_address  # Optionally return the IP address if needed elsewhere

    except Exception as e:
        print("WiFi connection failed:", e)
        
        # Rapid LED blink to indicate failure
        for _ in range(10):
            led.toggle()
            time.sleep(0.1)
        
        return None  # Return None to indicate a failed connection

# Example usage:
if __name__ == "__main__":
    ip = connect_wifi()
    if ip:
        print("Successfully connected to WiFi!")
    else:
        print("Failed to connect to WiFi. Check credentials or signal strength.")

