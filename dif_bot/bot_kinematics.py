from dif_bot import config  # Import the config module
from motor_pid.motor_velocity_pid import MotorVelocityPID  # Import PID control class
import time

class BotKinematics:
    def __init__(self):
        self.wheel_radius = config.WHEEL_RADIUS
        self.wheel_base = config.WHEEL_BASE
        self.controllers = self.initialize_motors()

    def initialize_motors(self):
        """Initialize all motors with their PID controllers and return a dictionary of controllers."""
        controllers = {}
        for motor_name, motor_config in config.MOTOR_PINS.items():
            pid_controller = MotorVelocityPID(
                motor_name=motor_config['motor_name'],
                input1=motor_config['input1'],
                input2=motor_config['input2'],
                encoder_a=motor_config['encoder_a'],
                encoder_b=motor_config['encoder_b'],
                gear_ratio=motor_config['gear_ratio'],
                encoder_resolution=motor_config['encoder_resolution'],
                kp=motor_config['pid']['kp'],
                ki=motor_config['pid']['ki'],
                kd=motor_config['pid']['kd']
            )
            controllers[motor_name] = pid_controller
            print(f"{motor_config['motor_name']} PID Controller Initialized Successfully.")
        return controllers

    def calculate_velocities(self, V, omega):
        """
        Calculates target velocities for each wheel based on desired linear (V) and angular (omega) velocities.
        """
        print(f"Linear Velocity: {V}, Angular Velocity: {omega}")
        
        # Extract parameters
        r = self.wheel_radius
        L = self.wheel_base / 2  # Half of the wheel base for differential drive calculations
        
        # Calculate each wheel's velocity
        left_velocity = (V - L * omega) / r
        right_velocity = (V + L * omega) / r
        
        return left_velocity, right_velocity
    
    def set_velocity(self, left_velocity, right_velocity):
        """Sets target velocities for each wheel in a differential drive configuration."""
        self.controllers["left"].set_target_velocity(-left_velocity)
        self.controllers["right"].set_target_velocity(right_velocity)
    
    def move(self, V, omega, duration):
        """Move the bot with specified velocities for a set duration."""
        
        # Calculate target velocities for each wheel
        left_velocity, right_velocity = self.calculate_velocities(V, omega)
        
        # Set velocities
        self.set_velocity(left_velocity, right_velocity)
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            for pid_controller in self.controllers.values():
                pid_controller.update()
            time.sleep(0.1)
        
        print("Stopping all motors.")
        self.stop()
        time.sleep(1)
            
    def update_motors(self):
        """runs the pid controller for each motor to reach its target velocity."""
        for pid_controller in self.controllers.values():
            pid_controller.update()

    def stop(self):
        """Stops all motors."""
        for pid_controller in self.controllers.values():
            pid_controller.stop()

if __name__ == "__main__":
     
    # Instantiate the bot kinematics
    bot = BotKinematics()
    # Test movements
   
    print("Forward")
    bot.move(V=0.5, omega=0.0, duration=2)  # Forward for 2 seconds    
    print("Back")
    bot.move(V=-0.5, omega=0.0, duration=2)  # Back for 2 seconds
    print("Rotate Left")
    bot.move(V=0.0, omega=3.0, duration=3)  # Rotate left for 3 seconds
    print("Rotate Right")
    bot.move(V=0.0, omega=3.0, duration=3)  # Rotate left for 3 seconds



