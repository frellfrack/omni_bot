from omni_bot import config  # Import the config module
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
            #print(f"{motor_config['motor_name']} PID Controller Initialized Successfully.")
        return controllers

    def calculate_velocities(self,Vx, Vy, omega):
        """
        Calculates target velocities for each wheel based on desired linear (Vx, Vy) and angular (omega) velocities.
        """        
        #print(f"vx:{Vx}, Vy:{Vy}, omega: {omega}")        
        # Extract parameters
        r = self.wheel_radius
        L = self.wheel_base
        # Calculate each wheel's velocity
        front_left_v = (Vy - Vx - (L * omega)) / r
        rear_left_v = (Vy + Vx - (L * omega)) / r
        front_right_v = (Vy + Vx + (L * omega)) / r
        rear_right_v = (Vy - Vx + (L * omega)) / r
        return front_left_v,rear_left_v,front_right_v,rear_right_v
    
    def set_velocity(self,front_left_v,rear_left_v,front_right_v,rear_right_v):
        """
        Sets target velocities for each wheel.
        """ 
        self.controllers["front_left"].set_target_velocity(-front_left_v)
        self.controllers["rear_left"].set_target_velocity(-rear_left_v)
        self.controllers["front_right"].set_target_velocity(front_right_v)
        self.controllers["rear_right"].set_target_velocity(rear_right_v)
    
    def move(self,Vx, Vy, omega, duration):
        
            #Calculate target velocities for each wheel
            front_left_v,rear_left_v,front_right_v,rear_right_v = self.calculate_velocities(Vx, Vy, omega)
                  
            """Move the bot with specified velocities for a set duration."""
            self.set_velocity(front_left_v,rear_left_v,front_right_v,rear_right_v)
            start_time = time.time()
            
            while time.time() - start_time < duration:
                for pid_controller in self.controllers.values():
                    pid_controller.update()
                time.sleep(0.1)

            #print("Stopping all motors.")
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
    
    while (True):
        print("Rotate Left")
        bot.move(Vx=0.0, Vy=0.0, omega=1, duration=3)
        print("Rotate Right")
        bot.move(Vx=0.0, Vy=0.0, omega=-1, duration=3)
     
