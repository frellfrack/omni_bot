import machine
import time
import math
class MotorVelocityPID:
    def __init__(self, motor_name, input1, input2, encoder_a, encoder_b, gear_ratio, encoder_resolution, kp, ki, kd):
        self.motor_name = motor_name
        self.gear_ratio = gear_ratio
        self.encoder_resolution = encoder_resolution
        self.counts_per_rev = gear_ratio * encoder_resolution
        
        # Motor control pins (PWM)
        self.motor_input1 = machine.PWM(machine.Pin(input1))
        self.motor_input2 = machine.PWM(machine.Pin(input2))
        self.motor_input1.freq(500)
        self.motor_input2.freq(500)

        # Encoder setup
        self.encoder_a = machine.Pin(encoder_a, machine.Pin.IN)
        self.encoder_b = machine.Pin(encoder_b, machine.Pin.IN)
        self.encoder_count = 0  # For position tracking
        self.prev_encoder_count = 0  # For velocity calculation
        self.prev_a = self.encoder_a.value()

        # PID parameters and variables
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_velocity = None  # Target velocity in rad/s
        self.target_position = None  # Target position in encoder counts
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.ticks_ms()

        # Attach encoder interrupt
        self.encoder_a.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=self._update_encoder)

    def _update_encoder(self, pin):
        """Update encoder counts for position."""
        a = self.encoder_a.value()
        b = self.encoder_b.value()
        if a != self.prev_a:
            increment = 1 if a == b else -1
            self.encoder_count += increment  # Update position count
        self.prev_a = a

    def set_target_velocity(self, target_velocity):
        """Set target velocity in rad/s."""
        self.target_velocity = target_velocity
        self.target_position = None  # Disable position control when in velocity mode
        self.integral = 0  # Reset integral when changing control mode

    def set_target_position(self, target_position):
        """Set target position relative to the current encoder count."""
        self.target_position = self.encoder_count + target_position
        self.target_velocity = None  # Disable velocity control when in position mode
        self.integral = 0  # Reset integral when changing control mode

    def get_actual_velocity(self):
        """Calculate and return the current velocity in rad/s."""
        current_time = time.ticks_ms()
        time_diff = time.ticks_diff(current_time, self.last_time) / 1000.0  # Time in seconds

        # Avoid division by zero
        if time_diff == 0:
            return 0

        # Calculate velocity based on encoder count difference
        count_diff = self.encoder_count - self.prev_encoder_count
        self.prev_encoder_count = self.encoder_count

        rotations = count_diff / self.counts_per_rev
        velocity = (rotations / time_diff) * 2 * 3.14159  # rad/s

        # Update last time after calculations
        self.last_time = current_time  # Ensure last_time is updated consistently
        return velocity

    def update(self):
        """Update motor control based on the active mode."""
        current_velocity = self.get_actual_velocity()
        desired_velocity = 0
        error = 0

        if self.target_position is not None:
            # Position control mode
            position_error = self.target_position - self.encoder_count
            if abs(position_error) < 5:  # Deadband to stop near the target
                self.stop()
                return
            desired_velocity = position_error * 0.1  # Scale factor to control the speed towards the target
        elif self.target_velocity is not None:
            # Velocity control mode
            desired_velocity = self.target_velocity
            if abs(error) < 0.1:  # Deadband for small velocity errors
                error = 0

        # Final error for PID control is the difference between desired and actual velocity
        error = desired_velocity - current_velocity

        # PID calculations
        current_time = time.ticks_ms()
        time_diff = time.ticks_diff(current_time, self.last_time) / 1000.0
        if abs(error) > 1.0:  # Prevent integral windup for small errors
            self.integral += error * time_diff
            # Clamp the integral to avoid windup
            self.integral = max(min(self.integral, 1000), -1000)
        self.derivative_history = getattr(self, 'derivative_history', [0] * 3)  # Moving average filter of size 3  # Moving average filter of size 5
        derivative_raw = (error - self.prev_error) / time_diff if time_diff > 0 else 0
        self.derivative_history.pop(0)
        self.derivative_history.append(derivative_raw)
        derivative = sum(self.derivative_history) / len(self.derivative_history)
        pid_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error

        # Convert PID output to PWM duty cycle
        duty_cycle = max(-65535, min(65535, int(pid_output * 65535 / 10)))
        self._set_pwm(duty_cycle)

    def _set_pwm(self, duty_cycle):
        """Set motor PWM duty cycle."""
        if duty_cycle > 0:
            self.motor_input1.duty_u16(duty_cycle)
            self.motor_input2.duty_u16(0)
        elif duty_cycle < 0:
            self.motor_input1.duty_u16(0)
            self.motor_input2.duty_u16(-duty_cycle)
        else:
            self.motor_input1.duty_u16(0)
            self.motor_input2.duty_u16(0)

    def stop(self):
        """Stops the motor."""
        self._set_pwm(0)


if __name__ == "__main__":
    
    def velocityPositionTest(motor, target_position, target_velocity):
        # Example for sequential position and velocity mode
        print ("")
        # Phase 1: Position Control
        motor.set_target_position(target_position)
        print(f"Testing {motor.motor_name} with target position: {target_position} counts")
        
        c = 0 
        while True:
            motor.update()
            time.sleep(0.05)
            current_position = motor.encoder_count
            print(f"try:{c}, Current position: {current_position} counts")
            if motor.target_position is not None and abs(current_position - motor.target_position) < 5:
                motor.stop()
                print("Position target reached.")
                break
            time.sleep(0.05)
            c = c + 1

        # Phase 2: Velocity Control
        motor.set_target_velocity(target_velocity)
        print(f"Testing {motor.motor_name} with target velocity: {target_velocity} rad/s")
        c = 0
        start_time = time.time()
        while time.time() - start_time < 5:  # Run velocity test for 5 seconds
            motor.update()
            time.sleep(0.05)
            av = motor.get_actual_velocity()
            print(f"try:{c}, Actual velocity: {av} rad/s")
            c = c + 1
            time.sleep(0.05)
        motor.stop()
        print("Velocity test completed.")
    
    def positionTest(motor, target_position):
        # Example for position  

        motor.set_target_position(target_position) 
        print(f"Testing {motor.motor_name} with target position: {target_position} counts")
        
        c = 0 
        while True:
            motor.update()
            current_position = motor.encoder_count
            print(f"try:{c}, Current position: {current_position} counts")
            if motor.target_position is not None and abs(current_position - motor.target_position) < 5:
                motor.stop()
                print("Position target reached.")
                break
            time.sleep(0.05)
            c = c + 1

    def velocityTest(motor, target_velocity, duration):
        # Example for velocity mode

        print(f"Testing {motor.motor_name} with target velocity: {target_velocity} rad/s")
        motor.set_target_velocity(target_velocity)
        c = 0
        start_time = time.time()
        while time.time() - start_time < duration:
            motor.update()
            av = motor.get_actual_velocity()
            c = c + 1
            print(f"try:{c}, Actual velocity: {av} rad/s")
            time.sleep(0.05)
        motor.stop()    
        
    motor_name = "Test Motor"
    input1 = 4
    input2 = 5
    encoder_a = 13
    encoder_b = 12
    gear_ratio = 270
    encoder_resolution = 128
    kp = 0.7
    ki = 0.001
    kd = 0.02

    pid_motor = MotorVelocityPID(
        motor_name, input1, input2, encoder_a, encoder_b, gear_ratio, encoder_resolution, kp, ki, kd
    )
    target_position = (4192 / 360) * (0.25 * 360)
    #velocityPositionTest(pid_motor, target_position, 2)
    #positionTest(pid_motor, target_position)
     
    while True:
        for i in range(80):  # Adjust the range to control the smoothness
            velocity = int(10 * math.sin(math.pi * i / 20))  # Scales sine wave to range -10 to 10
            velocityTest(pid_motor, velocity, 2)
