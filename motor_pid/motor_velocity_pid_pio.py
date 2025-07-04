import machine
import time
from motor_pid.pio_quadrature_encoder import PIOQuadratureEncoder  # Make sure this class is saved as a module

class MotorVelocityPID:
    def __init__(self, motor_name, input1, input2, encoder_a, encoder_b,
                 gear_ratio, encoder_resolution, kp, ki, kd, sm_id=0):
        self.motor_name = motor_name
        self.gear_ratio = gear_ratio
        self.encoder_resolution = encoder_resolution
        self.counts_per_rev = gear_ratio * encoder_resolution

        # Motor control pins (PWM)
        self.motor_input1 = machine.PWM(machine.Pin(input1))
        self.motor_input2 = machine.PWM(machine.Pin(input2))
        self.motor_input1.freq(500)
        self.motor_input2.freq(500)

        # Use PIO-based encoder
        self.encoder = PIOQuadratureEncoder(encoder_a, encoder_b, sm_id=sm_id)

        # PID parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_velocity = None
        self.target_position = None

        self.integral = 0
        self.prev_error = 0
        self.prev_encoder_count = 0
        self.last_time = time.ticks_ms()
        self.derivative_history = [0, 0, 0]

    def set_target_velocity(self, target_velocity):
        self.target_velocity = target_velocity
        self.target_position = None
        self.integral = 0

    def set_target_position(self, target_position):
        self.target_position = self.encoder.read() + target_position
        self.target_velocity = None
        self.integral = 0

    def get_actual_velocity(self):
        current_time = time.ticks_ms()
        time_diff = time.ticks_diff(current_time, self.last_time) / 1000.0  # seconds

        if time_diff == 0:
            return 0

        current_count = self.encoder.read()
        count_diff = current_count - self.prev_encoder_count
        self.prev_encoder_count = current_count

        rotations = count_diff / self.counts_per_rev
        velocity = (rotations / time_diff) * 2 * 3.14159  # rad/s

        self.last_time = current_time
        return velocity

    def update(self):
        current_velocity = self.get_actual_velocity()
        desired_velocity = 0

        if self.target_position is not None:
            current_position = self.encoder.read()
            position_error = self.target_position - current_position
            if abs(position_error) < 5:
                self.stop()
                return
            desired_velocity = position_error * 0.1  # simple proportional control
        elif self.target_velocity is not None:
            desired_velocity = self.target_velocity

        error = desired_velocity - current_velocity

        # PID control
        current_time = time.ticks_ms()
        time_diff = time.ticks_diff(current_time, self.last_time) / 1000.0

        if abs(error) > 1.0:
            self.integral += error * time_diff
            self.integral = max(min(self.integral, 1000), -1000)

        derivative_raw = (error - self.prev_error) / time_diff if time_diff > 0 else 0
        self.derivative_history.pop(0)
        self.derivative_history.append(derivative_raw)
        derivative = sum(self.derivative_history) / len(self.derivative_history)

        pid_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error

        # Scale and clamp PID output to PWM range
        duty_cycle = max(-65535, min(65535, int(pid_output * 65535 / 10)))
        self._set_pwm(duty_cycle)

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

    def stop(self):
        self._set_pwm(0)



if __name__ == "__main__":
    import math
    import time

    # Pin configuration (change these to match your hardware)
    motor_name = "PIO Test Motor"
    input1 = 4
    input2 = 5
    encoder_a = 13
    encoder_b = 12
    gear_ratio = 270
    encoder_resolution = 128
    kp = 1
    ki = 0.001
    kd = 0.02

    pid_motor = MotorVelocityPID(
        motor_name,
        input1,
        input2,
        encoder_a,
        encoder_b,
        gear_ratio,
        encoder_resolution,
        kp,
        ki,
        kd,
        sm_id=0  # Use state machine 0
    )

    def velocity_test(motor, velocity, duration_sec):
        print(f"\nVelocity test: {velocity} rad/s for {duration_sec} seconds")
        motor.set_target_velocity(velocity)
        start = time.time()
        while time.time() - start < duration_sec:
            motor.update()
            actual = motor.get_actual_velocity()
            print(f"Actual: {actual:.2f} rad/s")
            time.sleep(0.05)
        motor.stop()

    def position_test(motor, position_counts):
        print(f"\nPosition test: {position_counts} counts")
        motor.set_target_position(position_counts)
        while True:
            motor.update()
            current = motor.encoder.read()
            print(f"Position: {current} counts")
            if motor.target_position is not None and abs(current - motor.target_position) < 5:
                print("Target reached.")
                break
            time.sleep(0.05)
        motor.stop()

    def sine_wave_velocity_test(motor, duration=10, steps=100):
        print(f"\nSine wave test for {duration} seconds")
        start = time.time()
        i = 0
        while time.time() - start < duration:
            v = 10 * math.sin(math.pi * i / steps)
            print(f"Target velocity: {v:.2f} rad/s")
            velocity_test(motor, v, 0.3)
            i += 1

    # Run tests
    #target_position = int(0.25 * pid_motor.counts_per_rev)
    #position_test(pid_motor, target_position)
    velocity_test(pid_motor, 2.0, 5)
    #sine_wave_velocity_test(pid_motor, duration=8)
