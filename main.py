import uasyncio as asyncio
from machine import Timer
from omni_bot.bot_kinematics import BotKinematics
from webserver import WebServer
from websocket_router import WebSocketRouter
from wifi_manager import WiFiManager
import random

# === Wi-Fi Connection Logic ===
wifi = WiFiManager()
ip = wifi.auto_connect()

# If we reach here, connection was successful
print("Connected. IP address:", ip)

# === Bot & WebSocket Setup ===
bot = BotKinematics()
router = WebSocketRouter()

@router.route("ping")
def handle_ping(msg):
    print("[PING]")
    return {"type": "pong"}

@router.route("battery")
def handle_battery(msg):
    value = random.randint(1, 100)
    return {"type": "battery", "value": value}

@router.route("control")
def handle_control(msg):
    x = float(msg.get("x", 0))
    y = float(msg.get("y", 0))
    z = float(msg.get("z", 0))
    print(f"[CONTROL] X={x} Y={y} Z={z}")
    bot.set_velocity(*bot.calculate_velocities(x, y, z))

# === Timer & Motor Update Logic ===
motor_update_due = False

def motor_timer_callback(timer):
    global motor_update_due
    motor_update_due = True

motor_timer = Timer()
motor_timer.init(period=20, mode=Timer.PERIODIC, callback=motor_timer_callback)


def ws_client_connected():
    bot.stop()

def ws_client_disconnected():
    bot.stop()
    print("A WebSocket client has disconnected.")

def ws_client_error(e):
    bot.stop()
    print("WebSocket error:", e)


# === Main Async Loop ===
async def main():
    global motor_update_due

    bot.stop()

    server = WebServer(router=router,
    static_dir="web",
    on_disconnect=ws_client_disconnected,
    on_error=ws_client_error)
    await server.start()

    while True:
        if motor_update_due:
            bot.update_motors()
            motor_update_due = False
        await asyncio.sleep(0.005)

# === Run Main Event Loop ===
asyncio.run(main())
