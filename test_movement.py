import time
from config import ROBOT_ID
from robot.motion import RobotMotion
from robot.navigation import RobotNavigation
from Lidar.ld06_scan import LiDAR_Scan


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

def run_forward_with_lidar():
    motion = RobotMotion(robot_id = ROBOT_ID)
    lidar = LiDAR_Scan()
    navigation = RobotNavigation(ROBOT_ID, None, motion, lidar_scan=lidar)

    lidar.start()
    
    try:
        if lidar.get_latest_scan(wait=2.0) is None:
            print("No LiDAR scan received within 2 seconds. Exiting.")
            return
        start_time = time.monotonic()
        while time.monotonic() - start_time < 10.0:
            left, right = lidar.side_distance()
            print(f"Left distance: {left} mm, Right distance: {right} mm")
            navigation.drive_straight_with_lidar(left, right)
            time.sleep(0.05)  # Small delay to avoid overwhelming the system
    finally:
        lidar.close()
        print("\nStopping robot.")
        motion.stop()
if __name__ == "__main__":
    choice = int(input("1 for turntest, 2 for square test, 3 for forward test, 4 for rotate and forward test, 5 for forward test with LiDAR\n"))
    
    if choice == 1:
        turntest()
        
    if choice  == 2:
        run_square_test()
    if choice == 3:
        run_forward()
    if choice == 4:
        rotate_run_forward()
    if choice == 5:
        run_forward_with_lidar()
    
        