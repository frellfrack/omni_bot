# websocket_router.py

import ujson

class WebSocketRouter:
    def __init__(self):
        self.routes = {}

    def route(self, msg_type):
        """Register a handler function for a specific message type."""
        def decorator(func):
            self.routes[msg_type] = func
            return func
        return decorator

    def handle(self, msg):
        """Dispatch a decoded message to the appropriate handler."""
        msg_type = msg.get("type")
        if not msg_type:
            print("Message missing 'type' field")
            return None

        handler = self.routes.get(msg_type)
        if not handler:
            print(f"No handler registered for type: {msg_type}")
            return None

        try:
            response = handler(msg)
            if response:
                return ujson.dumps(response)
        except Exception as e:
            print(f"Error in handler for type '{msg_type}':", e)
        return None
