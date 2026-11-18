from jetbot import Robot
from robot.utils import get_voltage
import time



class RobotMotion:
    def __init__(self):
        self.robot = Robot()
        self.standard_voltage = 12.6  # Standard voltage for a fully charged battery

    def voltage_compensation_multiplier(self):
        current_voltage = get_voltage()
        if current_voltage < 9.0:  # If voltage is too low, avoid overcompensation
            current_voltage = 9.0  # Set to minimum safe voltage
        if current_voltage <= 0:  # Avoid division by zero
            return 1.0  # No compensation if voltage reading is invalid

        return self.standard_voltage / current_voltage

    def move_forward(self, speed=0.5, duration=1.0):
        self.robot.set_motors(speed, speed)
        time.sleep(duration)
        self.robot.stop()

    def move_backward(self, speed=0.5, duration=1.0):
        self.robot.set_motors(-speed, -speed)
        time.sleep(duration)
        self.robot.stop()

    def turn_90_left(self, speed=0.3, duration=1.0):
        multiplier = self.voltage_compensation_multiplier()
        adjusted_duration = duration * multiplier
        self.robot.set_motors(-speed, speed)
        time.sleep(adjusted_duration)
        self.robot.stop()

    def turn_90_right(self, speed=0.3, duration=1.0):
        multiplier = self.voltage_compensation_multiplier()
        adjusted_duration = duration * multiplier
        self.robot.set_motors(speed, -speed)
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
    
