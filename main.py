from server.connect_wifi import connect_wifi
from server.ws_server import start_websocket_server
from omni_bot.bot_kinematics import BotKinematics

if __name__ == "__main__":
  
    # Instantiate the bot
    bot = BotKinematics()
    bot.stop()
    
    # Connect to Wi-Fi
    ip_address = connect_wifi()
    print(f"Pico W is accessible at http://{ip_address}")

    # Start WebSocket server with bot control
    start_websocket_server(bot,ip_address)
