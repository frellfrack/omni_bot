# Bot configuration
WHEEL_RADIUS = 0.06  # in meters
WHEEL_BASE = 0.15    # half the diagonal distance between wheels ers)

# Motor Pin Assignments and Individual PID Settings
MOTOR_PINS = {
    "front_left": {   
        "motor_name": "Front Left",
        "input1": 3,
        "input2": 2,
        "encoder_a": 11,
        "encoder_b": 10,
        "gear_ratio": 131.3,
        "encoder_resolution": 64,
        "pid": {"kp": 10.56, "ki": 4.32, "kd": 0.02}
    },
    "rear_left": {   
        "motor_name": "Rear Left",
        "input1": 4,
        "input2": 5,
        "encoder_a": 13,
        "encoder_b": 12,
        "gear_ratio": 131.3,
        "encoder_resolution": 64,
        "pid": {"kp": 10.41, "ki": 4.94, "kd": 0.03}
    },
    "rear_right": {   
        "motor_name": "Rear Right",        
        "input1": 6,
        "input2": 7,
        "encoder_a": 18,
        "encoder_b": 19,
        "gear_ratio": 131.3,
        "encoder_resolution": 64,
        "pid": {"kp": 10.59, "ki": 4.99, "kd": 0.08}
    },
    "front_right": {
        "motor_name": "Front Right",
        "input1": 9,
        "input2": 8,
        "encoder_a": 20,
        "encoder_b": 21,
        "gear_ratio": 131.3,
        "encoder_resolution": 64,
        "pid": {"kp": 10.59, "ki": 4.92, "kd": 0.04}
    }
}
