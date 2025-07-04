# Bot configuration
WHEEL_RADIUS = 0.06  # in meters
WHEEL_BASE = 0.15    # half the diagonal distance between wheels ers)

# Motor Pin Assignments and Individual PID Settings
MOTOR_PINS = {
    "front_left": {   
        "motor_name": "Front Left",
        "input1": 2,
        "input2": 3,
        "encoder_a": 10,
        "encoder_b": 11,
        "gear_ratio": 131.3,
        "encoder_resolution": 64,
        "pid": {"kp": 4.1, "ki": 0, "kd": 0.01}
    },
    "rear_left": {   
        "motor_name": "Rear Left",
        "input1": 5,
        "input2": 4,
        "encoder_a": 12,
        "encoder_b": 13,
        "gear_ratio": 131.3,
        "encoder_resolution": 64,
        "pid": {"kp": 4.1, "ki": 1, "kd": 0.01}
    },
    "rear_right": {   
        "motor_name": "Rear Right",        
        "input1": 6,
        "input2": 7,
        "encoder_a": 18,
        "encoder_b": 19,
        "gear_ratio": 131.3,
        "encoder_resolution": 64,
        "pid": {"kp": 4.1, "ki": 0, "kd": 0.01}
    },
    "front_right": {
        "motor_name": "Front Right",
        "input1": 9,
        "input2": 8,
        "encoder_a": 20,
        "encoder_b": 21,
        "gear_ratio": 131.3,
        "encoder_resolution": 64,
        "pid": {"kp": 4.1, "ki": 0, "kd": 0.01}
    }
}


