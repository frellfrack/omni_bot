import time

from omni_bot.config import MOTOR_PINS
from motor_pid.motor_velocity_pid import MotorVelocityPID

# Autotune parameters
TARGET_VELOCITY = 1.5        # Desired motor speed in RPM for tuning
TUNING_DURATION = 3          # Duration in seconds for each tuning test
KP_RANGE = (0,6.0)      # Range of kp values to explore
KI_RANGE = (0,20.0)       # Range of ki values to explore
KD_RANGE = (0,0.20)       # Range of kd values to explore
KP_STEP = 0.25                # Step size for kp
KI_STEP = 0.25                # Step size for ki
KD_STEP = 0.0025                # Step size for kd

def initialize_motor(motor_config):
    """Initialize a single motor PID controller."""
    return MotorVelocityPID(
        motor_name=motor_config['motor_name'],
        input1=motor_config['input1'],
        input2=motor_config['input2'],
        encoder_a=motor_config['encoder_a'],
        encoder_b=motor_config['encoder_b'],
        gear_ratio=motor_config['gear_ratio'],
        encoder_resolution=motor_config['encoder_resolution'],
        kp=0.0,
        ki=0.0,
        kd=0.0
    )

def run_tuning_test(pid_controller, kp, ki, kd):
    """Run a tuning test with specific PID values and return the average error."""
    pid_controller.kp = kp
    pid_controller.ki = ki
    pid_controller.kd = kd

    pid_controller.set_target_velocity(TARGET_VELOCITY)
    total_error = 0.0
    iterations = 0
    start_time = time.time()

    while time.time() - start_time < TUNING_DURATION:
        pid_controller.update()
        actual_velocity = pid_controller.get_actual_velocity()
        error = abs(TARGET_VELOCITY - actual_velocity)
        total_error += error
        iterations += 1
        time.sleep(0.1)

    pid_controller.stop()
    return total_error / iterations if iterations > 0 else float('inf')

def tune_kp_only(pid_controller):
    """Tune kp with ki and kd set to zero."""
    best_kp = 0.0
    best_error = float('inf')

    kp = KP_RANGE[0]
    while kp <= KP_RANGE[1]:
        error = run_tuning_test(pid_controller, kp, 0.0, 0.0)
        print(f"Tuning kp: {kp:.2f}, Error: {error:.4f}")

        if error < best_error:
            best_kp = kp
            best_error = error

        kp += KP_STEP

    print(f"Best kp: {best_kp:.2f} with Error: {best_error:.4f}")
    return best_kp

def tune_ki_with_kp(pid_controller, kp):
    """Tune ki with fixed kp and kd set to zero."""
    best_ki = 0.0
    best_error = float('inf')

    ki = KI_RANGE[0]
    while ki <= KI_RANGE[1]:
        error = run_tuning_test(pid_controller, kp, ki, 0.0)
        print(f"Tuning ki with kp={kp:.2f} - ki: {ki:.2f}, Error: {error:.4f}")

        if error < best_error:
            best_ki = ki
            best_error = error

        ki += KI_STEP

    print(f"Best ki: {best_ki:.2f} with Error: {best_error:.4f}")
    return best_ki

def tune_kd_with_kp_ki(pid_controller, kp, ki):
    """Tune kd with fixed kp and ki."""
    best_kd = 0.0
    best_error = float('inf')

    kd = KD_RANGE[0]
    while kd <= KD_RANGE[1]:
        error = run_tuning_test(pid_controller, kp, ki, kd)
        print(f"Tuning kd with kp={kp:.2f}, ki={ki:.2f} - kd: {kd:.4f}, Error: {error:.4f}")

        if error < best_error:
            best_kd = kd
            best_error = error

        kd += KD_STEP

    print(f"Best kd: {best_kd:.2f} with Error: {best_error:.4f}")
    return best_kd

def autotune_motor(motor_name, motor_config):
    """Autotune the PID parameters for a motor in steps."""
    pid_controller = initialize_motor(motor_config)
    best_kp = tune_kp_only(pid_controller)
    best_ki = tune_ki_with_kp(pid_controller, best_kp)
    best_kd = tune_kd_with_kp_ki(pid_controller, best_kp, best_ki)

    print(f"Best PID for {motor_name} - kp: {best_kp:.2f}, ki: {best_ki:.2f}, kd: {best_kd:.3f}")
    return best_kp, best_ki, best_kd

def autotune_all_motors():
    """Autotune all motors based on config settings."""
    best_pid_values = {}
    for motor_name, motor_config in MOTOR_PINS.items():
        print(f"Autotuning {motor_name}...")
        best_pid_values[motor_name] = autotune_motor(motor_name, motor_config)

    print("Autotuning complete. Best PID values:")
    for motor_name, (kp, ki, kd) in best_pid_values.items():
        print(f"{motor_name} - kp: {kp:.2f}, ki: {ki:.2f}, kd: {kd:.2f}")

if __name__ == "__main__":
    autotune_all_motors()
