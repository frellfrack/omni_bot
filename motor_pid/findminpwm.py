import machine
import time
import math
from motor_pid.encoder import SimpleEncoder

def find_motor_characteristics(input1, input2, encoder_a, encoder_b,
                                gear_ratio, encoder_resolution,
                                pwm_freq=500, delay_ms=200, step=500,
                                hold_time=5):
    motor_input1 = machine.PWM(machine.Pin(input1))
    motor_input2 = machine.PWM(machine.Pin(input2))
    motor_input1.freq(pwm_freq)
    motor_input2.freq(pwm_freq)

    encoder = SimpleEncoder(encoder_a, encoder_b)
    counts_per_rev = gear_ratio * encoder_resolution

    def _set_pwm(duty_cycle):
        print(f"[PWM] Duty: {duty_cycle}")
        if duty_cycle > 0:
            motor_input1.duty_u16(duty_cycle)
            motor_input2.duty_u16(0)
        elif duty_cycle < 0:
            motor_input1.duty_u16(0)
            motor_input2.duty_u16(-duty_cycle)
        else:
            motor_input1.duty_u16(0)
            motor_input2.duty_u16(0)
    def _ramp_down(current_pwm, forward=True, step=1000, delay_ms=50):
        print("Ramping down...")
        for pwm in range(current_pwm, 0, -step):
            _set_pwm(pwm if forward else -pwm)
            time.sleep_ms(delay_ms)
        _set_pwm(0)
        
    def run_direction(forward=True):
        print(f"\n--- Testing {'forward' if forward else 'reverse'} direction ---")
        encoder.reset()
        initial_pos = encoder.read()
        min_pwm = None

        # Step 1: ramp up to find minimum effective PWM
        for pwm in range(0, 65536, step):
            _set_pwm(pwm if forward else -pwm)
            time.sleep_ms(delay_ms)
            new_pos = encoder.read()
            movement = new_pos - initial_pos
            print(f"PWM: {pwm:<5}  Δ Encoder: {movement}")

            if abs(movement) > 1:
                min_pwm = pwm
                print(f"Minimum movement detected at PWM {pwm}")
                break

        if min_pwm is None:
            print("No movement detected — skipping max speed test.")
            _set_pwm(0)
            return None, 0.0

        # Step 2: continue ramp to full power
        for pwm in range(min_pwm + step, 65536, step):
            _set_pwm(pwm if forward else -pwm)
            time.sleep_ms(delay_ms)

        # Step 3: hold at full speed and measure max rad/s
        print(f"Holding max PWM for {hold_time}s to measure speed...")
        max_rad_s = 0.0
        start_time = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start_time) < hold_time * 1000:
            cps = encoder.velocity()
            rad_s = (cps / counts_per_rev) * 2 * math.pi
            if abs(rad_s) > abs(max_rad_s):
                max_rad_s = rad_s
            print(f"Measured speed: {rad_s:.2f} rad/s")
            time.sleep(0.1)

        # Step 4: stop motor
        _ramp_down(65535, forward)
        return min_pwm, max_rad_s

    # Test both directions
    min_fwd, max_fwd = run_direction(forward=True)
    print("Resting for 1 second before reversing...")
    _set_pwm(0)
    time.sleep(1)
    min_rev, max_rev = run_direction(forward=False)

    # Final report
    print("\n===== Motor Test Results =====")
    print(f"Min Forward PWM : {min_fwd if min_fwd is not None else 'Not detected'}")
    print(f"Max Forward Speed: {max_fwd:.2f} rad/s")
    print(f"Min Reverse PWM : {min_rev if min_rev is not None else 'Not detected'}")
    print(f"Max Reverse Speed: {max_rev:.2f} rad/s")

    return {
        "min_pwm_forward": min_fwd,
        "max_rad_s_forward": max_fwd,
        "min_pwm_reverse": min_rev,
        "max_rad_s_reverse": max_rev
    }

# Example usage
if __name__ == "__main__":
    find_motor_characteristics(
        input1=2, input2=3,
        encoder_a=10, encoder_b=11,
        gear_ratio=131.3, encoder_resolution=64
    )
