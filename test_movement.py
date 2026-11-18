import time

from robot.motion import RobotMotion

robot = RobotMotion()
times = int(input("Enter how many times to turn 90 degrees left: "))
time.sleep(1)

for _ in range(times):
    robot.turn_90_left(speed=0.3, duration=1.0)