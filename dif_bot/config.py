# Dif Bot configuration
WHEEL_RADIUS = 0.06  # in meters
WHEEL_BASE = 0.30    # distance between wheels 

# Motor Pin Assignments and Individual PID Settings
MOTOR_PINS = {
    "left": {   
        "motor_name": "Left Wheel",
        "input1": 3,
        "input2": 2,
        "encoder_a": 11,
        "encoder_b": 10,
        "gear_ratio": 270,
        "encoder_resolution": 64,
        "pid": {"kp": 10.56, "ki": 4.32, "kd": 0.02}
    },
    "right": {   
        "motor_name": "Right Wheel",
        "input1": 4,
        "input2": 5,
        "encoder_a": 13,
        "encoder_b": 12,
        "gear_ratio": 270,
        "encoder_resolution": 64,
        "pid": {"kp": 10.41, "ki": 4.94, "kd": 0.03}
    }
}
