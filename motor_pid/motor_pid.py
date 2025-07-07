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

        # PID parameters and state
        self.set_pid(kp, ki, kd)
        self.target_velocity = None
        self.target_position = None
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.ticks_ms()

    def set_pid(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_kp(self, kp): self.kp = kp
    def set_ki(self, ki): self.ki = ki
    def set_kd(self, kd): self.kd = kd
    def get_pid(self): return (self.kp, self.ki, self.kd)
    def get_counts_per_rev(self): return self.counts_per_rev

    def set_target_velocity(self, velocity_rads):
        self.target_velocity = velocity_rads
        self.target_position = None

    def set_target_position(self, encoder_counts):
        self.target_position = encoder_counts
        self.target_velocity = None

    def set_target_position_and_velocity(self, encoder_counts, velocity_rads):
        self.target_position = encoder_counts
        self.target_velocity = velocity_rads

    def get_position(self):
        return self.encoder.get_position()

    def get_velocity(self):
        counts_per_sec = self.encoder.get_velocity()
        return (counts_per_sec / self.counts_per_rev) * (2 * 3.14159265359)

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
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time) / 1000.0

        if self.target_position is not None and self.target_velocity is not None:
            self._do_position_with_velocity_cap(dt)
        elif self.target_position is not None:
            self._do_position(dt)
        elif self.target_velocity is not None:
            self._do_velocity(dt)

        self.last_time = now

    def _do_velocity(self, dt):
        measured_velocity = self.get_velocity()
        error = self.target_velocity - measured_velocity

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        max_pwm = 65535
        output = max(-max_pwm, min(max_pwm, int(output)))

        self._set_pwm(output)
        self.prev_error = error

    def _do_position(self, dt):
        measured_position = self.get_position()
        error = self.target_position - measured_position

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        max_pwm = 65535
        output = max(-max_pwm, min(max_pwm, int(output)))

        self._set_pwm(output)
        self.prev_error = error

    def _do_position_with_velocity_cap(self, dt):
        pos_error = self.target_position - self.get_position()
        self.integral += pos_error * dt
        derivative = (pos_error - self.prev_error) / dt if dt > 0 else 0
        velocity_setpoint = self.kp * pos_error + self.ki * self.integral + self.kd * derivative

        max_vel = abs(self.target_velocity)
        velocity_setpoint = max(-max_vel, min(max_vel, velocity_setpoint))

        measured_velocity = self.get_velocity()
        vel_error = velocity_setpoint - measured_velocity

        self.integral += vel_error * dt
        vel_derivative = (vel_error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * vel_error + self.ki * self.integral + self.kd * vel_derivative

        max_pwm = 65535
        output = max(-max_pwm, min(max_pwm, int(output)))

        self._set_pwm(output)
        self.prev_error = vel_error

if __name__ == "__main__":
    from omni_bot import config

    motor = MotorPID(
        motor_name="test_motor",
        input1=0, input2=1,
        encoder_a=2, encoder_b=3,
        gear_ratio=30, encoder_resolution=12,
        kp=5.0, ki=1.0, kd=0.4
    )

    print("Testing velocity mode...")
    motor.set_target_velocity(2.0)
    for _ in range(50):
        motor.update()
        time.sleep(0.1)

    print("Testing position mode...")
    motor.set_target_position(100)
    for _ in range(50):
        motor.update()
        time.sleep(0.1)

    print("Testing position with velocity cap...")
    motor.set_target_position_and_velocity(200, 2.0)
    for _ in range(50):
        motor.update()
        time.sleep(0.1)
