from jetbot import Robot
from robot.utils import get_voltage
import time



class RobotMotion:
    def __init__(self):
        self.robot = Robot()
        self.standard_voltage = 11.796 #Standard voltage
        self.left_trim = 1.0
        self.right_trim = 0.885
    def voltage_compensation_multiplier(self):
        current_voltage = get_voltage()
        if current_voltage < 9.0:  # If voltage is too low, avoid overcompensation
            current_voltage = 9.0  # Set to minimum safe voltage
        if current_voltage <= 0:  # Avoid division by zero
            return 1.0  # No compensation if voltage reading is invalid

        return self.standard_voltage / current_voltage
    def smooth_stop(self, left_speed, right_speed, steps=3, delay=0.015):
        for i in range(steps, 0, -1):
            factor = i /steps
            
            adjusted_right = right_speed * factor
            adjusted_left = left_speed * factor*0.69
            self.robot.set_motors(adjusted_right, adjusted_left)
            time.sleep(delay)
        self.robot.stop()

    def move_forward(self, speed=0.2, duration=1.0):
        left_speed = speed * self.left_trim
        right_speed = speed * self.right_trim
        self.robot.set_motors(left_speed, right_speed)
        time.sleep(duration)
        self.smooth_stop(left_speed, right_speed)

    def move_backward(self, speed=0.5, duration=1.0):
        self.robot.set_motors(-speed, -speed)
        time.sleep(duration)
        self.robot.stop()

    def turn_90_left(self, speed=0.3, duration=0.3):
        multiplier = self.voltage_compensation_multiplier()
        current_voltageyes = get_voltage()
        
        print(f"voltage: {current_voltageyes}V")
        adjusted_duration = duration * multiplier
        
        left_speed = -speed
        right_speed = speed
        self.robot.set_motors(left_speed, right_speed)
        time.sleep(adjusted_duration-0.045)
        self.smooth_stop(left_speed, right_speed)

    def turn_90_right(self, speed=0.3, duration=0.21):
        #multiplier = self.voltage_compensation_multiplier()
        current_voltageyes2 = get_voltage()
        multiplier = 1.0
        print(f"voltage: {current_voltageyes2}V")
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
    
