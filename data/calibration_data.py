
ROBOT_CALIBRATIONS = {
    "default": {
        "left_motor_alpha":1.0,
        "right_motor_alpha":1.0,
        "left_motor_beta":0.0,
        "right_motor_beta":0.0,
        "turn_left_duration": 0.236,
        "turn_left_speed": 0.3,
        "turn_right_duration": 0.233,
        "turn_right_speed": 0.3,
        "standard_voltage": 12.0
    },
    #Works alright tested on 1 meter little error near end(black)
    "robot_1": {
        "left_motor_alpha": 1.077,
        "right_motor_alpha": 0.945,
        "left_motor_beta": 0.0,
        "right_motor_beta": 0.0,
        "turn_left_duration": 0.236,
        "turn_left_speed": 0.3,
        "turn_right_duration": 0.233,
        "turn_right_speed": 0.3,
        "standard_voltage": 12.0
    },
    
    
    
}


def get_robot_calibration(robot_id="default"):
    base = ROBOT_CALIBRATIONS["default"].copy()
    specific = ROBOT_CALIBRATIONS.get(robot_id, {})
    base.update(specific)
    return base