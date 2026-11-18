import time

from robot.motion import RobotMotion
def turntest():

    
    robot = RobotMotion()
    times = int(input("Enter how many times to turn 90 degrees left: "))
    time.sleep(1)

    for _ in range(times):
        robot.turn_90_left()
        time.sleep(0.5)

def run_square_test():
    print("square test")
    motion = RobotMotion()
    
    for i in range(1, 5):
        motion.move_forward(duration=2)
        
        time.sleep(0.5)
        
        motion.turn_90_left(speed=0.3)
        
        time.sleep(0.5)
        
def run_forward():
    motion = RobotMotion()
    motion.move_forward( duration=4)
        
if __name__ == "__main__":
    choise = int(input("1 for turntest, 2 for square test\n"))
    
    if choise == 1:
        turntest()
        
    if choise == 2:
        run_square_test()
    if choise == 3:
        run_forward()