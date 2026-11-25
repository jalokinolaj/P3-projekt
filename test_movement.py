import time
from config import ROBOT_ID
from robot.motion import RobotMotion


FLOOR_PROFILE = "woodenfloor"


def turntest():
    robot = RobotMotion(robot_id = ROBOT_ID)
    times = int(input("Enter how many times to turn 90 degrees left: "))
    time.sleep(1)

    for _ in range(times):
        robot.turn_90_right()
        time.sleep(0.5)

def run_square_test():
    print("square test")
    motion = RobotMotion(robot_id = ROBOT_ID)
    
    for i in range(1, 5):
        motion.move_forward(speed=0.2)
        time.sleep(2.0)
        motion.stop()
        
        time.sleep(0.5)
        
        motion.turn_90_left()
        
        time.sleep(0.5)
        
def run_forward():
    motion = RobotMotion(robot_id = ROBOT_ID)
    motion.move_forward(speed=0.2)
    time.sleep(5.0)
    motion.stop()
def rotate_run_forward():
    motion = RobotMotion(robot_id = ROBOT_ID)
    motion.turn_90_left()
    motion.move_forward(speed=0.2)
    time.sleep(5.0)
    motion.stop()
if __name__ == "__main__":
    choise = int(input("1 for turntest, 2 for square test, 3 for forward test\n"))
    
    if choise == 1:
        turntest()
        
    if choise == 2:
        run_square_test()
    if choise == 3:
        run_forward()
    if choise == 4:
        rotate_run_forward()
        