from jetbot import Robot
from robot.utils import get_voltage
import time
import json
from pathlib import Path



class RobotMotion:
    def __init__(self, floor="default", calibration_path=None):
        self.robot = Robot()
        self.calibration_path = (
            Path(calibration_path)
            if calibration_path
            else Path(__file__).with_name("motion_calibration.json")
        )
        self.floor = floor
        self.standard_voltage = 11.796
        self.left_correction = 1.0
        self.right_correction = 1.0
        self.left_turn_duration = 0.30
        self.right_turn_duration = 0.21
        self.load_calibration(floor)

    def load_calibration(self, floor=None):
        if floor is not None:
            self.floor = floor

        try:
            with self.calibration_path.open("r", encoding="utf-8") as calibration_file:
                profiles = json.load(calibration_file)
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            profiles = {}

        profile = profiles.get(self.floor, profiles.get("default", {}))
        self.standard_voltage = float(profile.get("standard_voltage", 11.796))
        self.left_correction = float(profile.get("left_correction", 1.0))
        self.right_correction = float(profile.get("right_correction", 0.885))
        self.left_turn_duration = float(profile.get("left_turn_duration", 0.30))
        self.right_turn_duration = float(profile.get("right_turn_duration", 0.21))

    def set_floor(self, floor):
        self.load_calibration(floor)

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

    def move_forward(self, speed=0.2, duration=1.0):
        left_speed = speed * self.left_correction
        right_speed = speed * self.right_correction
        self.robot.set_motors(left_speed, right_speed)
        time.sleep(duration)
        self.smooth_stop(left_speed, right_speed)

    def move_backward(self, speed=0.5, duration=1.0):
        left_speed = -speed * self.left_correction
        right_speed = -speed * self.right_correction
        self.robot.set_motors(left_speed, right_speed)
        time.sleep(duration)
        self.smooth_stop(left_speed, right_speed)

    def turn_90_left(self, speed=0.3, duration=None):
        multiplier = self.voltage_compensation_multiplier()
        if duration is None:
            duration = self.left_turn_duration
        adjusted_duration = duration * multiplier

        left_speed = -speed
        right_speed = speed
        self.robot.set_motors(left_speed, right_speed)
        time.sleep(adjusted_duration)
        self.robot.stop()

    def turn_90_right(self, speed=0.3, duration=None):
        multiplier = self.voltage_compensation_multiplier()
        if duration is None:
            duration = self.right_turn_duration
        adjusted_duration = duration * multiplier
        left_speed = speed
        right_speed = -speed
        self.robot.set_motors(left_speed, right_speed)
        time.sleep(adjusted_duration)
        self.robot.stop()

    def nudge_left(self, speed=0.2, duration=0.5):
        self.robot.set_motors(-speed, speed)
        time.sleep(duration)
        self.robot.stop()

    def nudge_right(self, speed=0.2, duration=0.5):
        self.robot.set_motors(speed, -speed)
        time.sleep(duration)
        self.robot.stop()
    
