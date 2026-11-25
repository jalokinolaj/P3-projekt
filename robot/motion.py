from jetbot import Robot
from robot.utils import get_voltage
from data.calibration_data import *
import time
import json
from pathlib import Path



class RobotMotion:
    def __init__(self, robot_id="default"):
        self.robot_id = robot_id
        self.robot = Robot()
        self.cal = get_robot_calibration(self.robot_id)
        self.standard_voltage = self.cal["standard_voltage"]
        
        self.robot.left_motor.alpha = self.cal.get("left_motor_alpha", 1.0)
        self.robot.right_motor.alpha = self.cal.get("right_motor_alpha", 1.0)
        self.robot.left_motor.beta = self.cal.get("left_motor_beta", 0.0)
        self.robot.right_motor.beta = self.cal.get("right_motor_beta", 0.0)
        self.turn_left_speed = self.cal.get("turn_left_speed", 0.3)
        self.turn_left_duration = self.cal.get("turn_left_duration", 0.25)
        self.turn_right_speed = self.cal.get("turn_right_speed", 0.3)
        self.turn_right_duration = self.cal.get("turn_right_duration", 0.25)
        self.standard_voltage = self.cal.get("standard_voltage", 12.0)

       

    def voltage_compensation_multiplier(self):
        current_voltage = get_voltage()
        if current_voltage <= 0:
            return 1.0  # No compensation if voltage reading is invalid
        if current_voltage < 9.0:
            current_voltage = 9.0

        return self.standard_voltage / current_voltage

    def smooth_stop(self, left_speed, right_speed, steps=3, delay=0.015):
        for i in range(steps, 0, -1):
            factor = i / steps
            self.robot.set_motors(left_speed * factor, right_speed * factor)
            time.sleep(delay)
        self.robot.stop()

    def stop(self):
        self.robot.stop()

    def move_forward(self, speed=0.2):
        self.robot.set_motors(speed, speed)

    def move_with_speeds(self, left_speed, right_speed):
        self.robot.set_motors(left_speed, right_speed)

    def move_backward(self, speed=0.2):
        self.robot.set_motors(-speed, -speed)

    def turn_90_left(self):
        multiplier = self.voltage_compensation_multiplier()
        adjusted_duration = self.turn_left_duration * multiplier

        left_speed = -self.turn_left_speed
        right_speed = self.turn_left_speed
        self.robot.set_motors(left_speed, right_speed)
        time.sleep(adjusted_duration)
        self.robot.stop()

    def turn_90_right(self):
        multiplier = self.voltage_compensation_multiplier()
        adjusted_duration = self.turn_right_duration * multiplier
        left_speed = self.turn_right_speed
        right_speed = -self.turn_right_speed
        self.robot.set_motors(left_speed, right_speed)
        time.sleep(adjusted_duration)
        self.robot.stop()

    def nudge_left(self, speed=0.2, duration=0.08):
        self.robot.set_motors(-speed, speed)
        time.sleep(duration)
        self.robot.stop()

    def nudge_right(self, speed=0.2, duration=0.08):
        self.robot.set_motors(speed, -speed)
        time.sleep(duration)
        self.robot.stop()
    
