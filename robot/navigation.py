import time
import logging
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot.motion import RobotMotion
    from robot.vision import VisionSystem

class RobotNavigation:
    GRACEPERIODDELAY = 3.0  # Grace period in seconds before notifying a delay
    OPEN_DISTANCE_THRESHOLD = 400  # Distance threshold in millimeters to consider a side as open
    HARD_TIMEOUT_FACTOR = 3  # Factor to multiply the estimated duration for hard timeout
    DRIVE_SPEED = 0.2  # Default drive speed for the robot
    DEFAULT_STEP_SECONDS = 1.0  # Default duration for each step in seconds
    def __init__(
        self,
        robot_id,
        vision_system: "VisionSystem",
        robot_motion: "RobotMotion",
        on_node_arrived=None,
        on_error=None,
    ):
        self.robot_id = robot_id
        self.vision = vision_system
        self.motion = robot_motion
    
        self.on_node_arrived = on_node_arrived
        self.on_error = on_error
        self.current_route = None
        self.current_queue = []
        self.pending_routes = []
        self.current_state = "IDLE"  # Track the robot's current state: "IDLE", "MOVING", or "QUEUED"
        self.current_direction = "NORTH"  # Track the robot's current direction: "NORTH", "EAST", "SOUTH", or "WEST"
        self.current_node = None
        self.goal_node = None  # Track the goal node for the current route
        self.current_route_step = 0  # Track the current step in the route
        self.hold_requested = threading.Event()

    def request_hold_at_next_node(self):
        self.hold_requested.set()
        if self.current_state == "IDLE":
            self.current_state = "HOLDING"

    def clear_hold_request(self):
        self.hold_requested.clear()

    def _hold_if_requested(self):
        if not self.hold_requested.is_set():
            return False
        # This check runs at node boundaries, so an edge is never cut short.
        self.current_route = None
        self.current_queue.clear()
        self.current_state = "HOLDING"
        return True

    def navigate_route(self, path):
        """Navigate the robot along the specified path."""
        self.current_route = path
        if not path:
            self.current_route = None
            self.current_state = "IDLE"
            return

        self.current_state = "MOVING"
        self.current_queue = list(path)
        goal_node = path[-1][0]
        self.goal_node = goal_node
        prev_eta = None
        self.current_path = path


        while self.current_queue:
            # Route steps include waits as well as moves.
            self.current_route_step = len(self.current_path)-len(self.current_queue)
            step = self.current_queue.pop(0)
            next_node, target_direction, next_eta = step
            if next_node is None:
                logging.warning(f"Robot {self.robot_id} has no next node to navigate to.")
                continue

            if prev_eta is None:
                self.current_node = next_node
                prev_eta = next_eta
                if self._hold_if_requested():
                    return
                continue

            if target_direction:
                self.turn_to_direction(self.current_direction, target_direction)

            if next_node == self.current_node:
                time.sleep(max(0, next_eta - prev_eta))  # Wait until the next estimated time of arrival
                prev_eta = next_eta
                self.notify_node_arrived(next_node)
                if self._hold_if_requested():
                    return
                continue

            duration_estimate = self.DEFAULT_STEP_SECONDS
            if prev_eta is not None and next_eta is not None:
                duration_estimate = max(0, next_eta - prev_eta)

            is_final_goal = len(self.current_queue) == 0
            if is_final_goal:
                found = self.drive_forward_until_goaltag(duration_estimate=duration_estimate)
                if not found:
                    self.handle_navigation_error(next_node)
                    return
                self.notify_node_arrived(next_node, is_goal=True)
                prev_eta = next_eta
            else:
                found = self.drive_forward_until_intersection_node(next_node=next_node, duration_estimate=duration_estimate)
                if not found:
                    self.handle_navigation_error(next_node)
                    return
                self.current_node = next_node
                prev_eta = next_eta
                self.notify_node_arrived(next_node)
            if self._hold_if_requested():
                return
                

        self.current_route = None
        if self._hold_if_requested():
            return
        self.current_state = "IDLE"

        


    def turn_to_direction(self, current_direction, target_direction):
        """Turn the robot to face the target direction."""
        directions = ["NORTH", "EAST", "SOUTH", "WEST"]
        current_direction = current_direction.upper()
        target_direction = target_direction.upper()

        try:
            old_index = directions.index(current_direction)
            new_index = directions.index(target_direction)
            delta = (new_index - old_index) % 4

            if delta == 1:
                self.motion.turn_90_right()
            elif delta == 2:
                self.motion.turn_90_right()
                self.motion.turn_90_right()
            elif delta == 3:
                self.motion.turn_90_left()
        except ValueError as error:
            logging.error(
                "Robot %s encountered an unknown direction: %s",
                self.robot_id,
                error,
            )
        else:
            self.current_direction = target_direction

    def notify_node_arrived(self, node, is_goal=False):
        """Notify the client that the robot has arrived at a node."""
        self.current_node = node
        if self.on_node_arrived is not None:
            self.on_node_arrived(
                node,
                is_goal=is_goal,
                route_step=self.current_route_step,
            )

    def drive_forward_until_intersection_node(self, next_node, duration_estimate):
        """Drive to the next node, then publish that arrival."""
        max_wait = duration_estimate * self.HARD_TIMEOUT_FACTOR+ self.GRACEPERIODDELAY
        self.motion.move_forward(speed = self.DRIVE_SPEED)
        start_time = time.monotonic()
        seen_intersection = False  # Track whether an intersection has been detected
        
        try:
            while True:
                elapsed_time = time.monotonic() - start_time
                left_distance, right_distance = self.motion.get_LiDAR_readings()


                if left_distance is not None and right_distance is not None:
                    is_open = left_distance > self.OPEN_DISTANCE_THRESHOLD or right_distance > self.OPEN_DISTANCE_THRESHOLD

                    if not is_open:
                        seen_intersection = True
                    elif seen_intersection:
                        logging.info(f"Robot {self.robot_id} detected an intersection. Stopping.")
                        return True  # Indicate that the robot has reached the intersection/node

                if elapsed_time > max_wait:
                    logging.error(f"Robot {self.robot_id} exceeded maximum wait time of {max_wait:.2f} seconds. Stopping.")
                    return False  # Indicate that the goal tag was not detected within the maximum wait time

                time.sleep(0.05)  # Prevent busy waiting
        finally:
            self.motion.stop()
            duration = time.monotonic() - start_time
            logging.info(
                f"Robot {self.robot_id} arrived at node {next_node}. At duration: {duration:.2f} seconds."
            )

    def run_routes(self):
        while self.pending_routes:
            self.navigate_route(self.pending_routes.pop(0))

    def drive_forward_until_goaltag(self, duration_estimate):
        """Drive the robot forward until it detects the goal AprilTag."""
        self.vision.reset_tag_detection()  # Reset the tag detection state before starting

        self.motion.move_forward(speed = self.DRIVE_SPEED)  # Move forward with a duration estimate
        start_time = time.monotonic()
        
        max_wait = duration_estimate * self.HARD_TIMEOUT_FACTOR + self.GRACEPERIODDELAY
        try:
            while True:
                tag = self.vision.get_latest_tag()
                if tag is not None:
                    logging.info(f"Robot {self.robot_id} detected AprilTag ID: {tag}. Stopping.")
                    return True  # Indicate that the goal tag was detected
                
                elapsed_time = time.monotonic() - start_time
                if elapsed_time > max_wait:
                    logging.error(f"Robot {self.robot_id} exceeded maximum wait time of {max_wait:.2f} seconds. Stopping.")
                    return False  # Indicate that the goal tag was not detected within the maximum wait time
                
                time.sleep(0.1)
        finally:
            self.motion.stop()
            logging.info(f"Robot {self.robot_id} has stopped.")


    def notify_error(self, last_known_node, expected_node):
        """Notify the client that the robot has encountered an error."""
        if self.on_error is not None:
            self.on_error(last_known_node, expected_node)
    def handle_navigation_error(self, expected_node):
        """Handle navigation errors by notifying the client and stopping the robot."""
        logging.error(f"Robot {self.robot_id} failed to reach expected node: {expected_node}. Stopping navigation.")
        self.motion.stop()
        self.current_queue.clear()  # Clear the current queue to stop further navigation
        self.pending_routes.clear()  # Clear any pending routes
        self.current_route = None  # Clear the current route
        self.current_state = "ERROR"  # Update state to ERROR
        self.notify_error(self.current_node, expected_node)
  

    def check_for_side_distance(self):
        """Check for side distances using LiDAR readings."""
        self.LiDAR_left, self.LiDAR_right = self.motion.get_LiDAR_readings()
        return self.LiDAR_left, self.LiDAR_right

        

        


