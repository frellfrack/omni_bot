import machine
from motor_pid.encoder import SimpleEncoder
import time

class MotorPID:
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
        self.encoder = SimpleEncoder(encoder_a, encoder_b)

        # PID parameters and variables
        self.set_pid(kp, ki, kd)
        self.target_velocity = None  # Target velocity in rad/s
        self.target_position = None  # Target position in encoder counts
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.ticks_ms()
    
    def set_pid(self, kp, ki, kd):
        """Public method to set PID gains."""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_kp(self, kp):
        """Set proportional gain."""
        self.kp = kp
    
    def set_ki(self, ki):
        """Set integral gain."""
        self.ki = ki
    
    def set_kd(self, kd):
        """Set derivative gain."""
        self.kd = kd
    
    def get_pid(self):
        """Get current PID gains as a tuple."""
        return (self.kp, self.ki, self.kd)
    
    def get_counts_per_rev(self):
        """Returns the number of encoder counts per revolution."""
        return self.counts_per_rev
 
    def set_target_velocity(self, velocity_rads):
        """Set target motor velocity in radians per second."""
        self.target_velocity = velocity_rads
    
    def get_velocity(self):
        """Returns the current motor velocity in radians per second."""
        return self._current_velocity_rads
    
    def set_target_position(self, encoder_counts):
        """Set target motor position in encoder counts."""
        self.target_position = encoder_counts
    
    def get_position(self):
        """Returns the current motor position in encoder counts."""
        return self._current_position_counts
    
    def set_target_position_and_velocity(self, encoder_counts, velocity_rads):
        """Set both target position (encoder counts) and max velocity (rad/s)."""
        self.target_position = encoder_counts
        self.target_velocity = velocity_rads

    def _set_pwm(self, duty_cycle):
        print(f"[PWM] Duty: {duty_cycle}")
        if duty_cycle > 0:
            self.motor_input1.duty_u16(duty_cycle)
            self.motor_input2.duty_u16(0)
        elif duty_cycle < 0:
            self.motor_input1.duty_u16(0)
            self.motor_input2.duty_u16(-duty_cycle)
        else:
            self.motor_input1.duty_u16(0)
            self.motor_input2.duty_u16(0)

    def update(self):
        """Update the motor control based on PID calculations."""
          # Calculate time delta
        now = time.ticks_ms()
        dt = (time.ticks_diff(now, self.last_time)) / 1000.0  # seconds
        if (self.target_position is not None and self.target_velocity is not None):
            # handle position with velocity cap
            self._do_position_with_velocity_cap(now,dt)
        elif(self.target_position is not None):
            self._do_position()
        elif(self.target_velocity is not None):
            self._do_velocity()
        self.last_time = now
    def _do_velocity(self,now,dt):
        """Handle velocity control (PID on velocity error, output to PWM)."""

        # Get measured velocity
        measured_velocity = self.get_velocity()
        error = self.target_velocity - measured_velocity

        # PID calculations
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Clamp output to PWM range
        max_pwm = 65535
        output = max(-max_pwm, min(max_pwm, int(output)))

        self._set_pwm(output)
        self.prev_error = error
   
    def _do_position(self, now, dt):
        """Handle position control (PID on position error, output to PWM)."""
        # Get measured position
        measured_position = self.get_position()
        error = self.target_position - measured_position

        # PID calculations
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Clamp output to PWM range
        max_pwm = 65535
        output = max(-max_pwm, min(max_pwm, int(output)))

        self._set_pwm(output)
        self.prev_error = error

    def _do_position_with_velocity_cap(self, now, dt):
        """Handle position control with velocity cap (all velocities in rad/s, fully independent)."""
        # 1. Compute position error (encoder counts)
        measured_position = self.get_position()
        pos_error = self.target_position - measured_position

        # 2. Position PID outputs velocity setpoint (rad/s)
        self.integral += pos_error * dt
        derivative = (pos_error - self.prev_error) / dt if dt > 0 else 0
        velocity_setpoint = self.kp * pos_error + self.ki * self.integral + self.kd * derivative

        # 3. Cap velocity setpoint to ±self.target_velocity (rad/s)
        max_vel = abs(self.target_velocity)
        if velocity_setpoint > max_vel:
            velocity_setpoint = max_vel
        elif velocity_setpoint < -max_vel:
            velocity_setpoint = -max_vel

        # 4. Calculate velocity error (rad/s)
        measured_velocity = self.get_velocity()
        vel_error = velocity_setpoint - measured_velocity

        # 5. PID on velocity error (independent from other methods)
        # Use separate integral/prev_error for this loop if needed, but for now, use the same
        self.integral += vel_error * dt
        vel_derivative = (vel_error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * vel_error + self.ki * self.integral + self.kd * vel_derivative

        # Clamp output to PWM range
        max_pwm = 65535
        output = max(-max_pwm, min(max_pwm, int(output)))

        self._set_pwm(output)
        self.prev_error = vel_error
       