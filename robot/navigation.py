import time
import logging
import threading
from typing import TYPE_CHECKING
from robot.vision import VisionSystem
from robot.client import RobotClient

if TYPE_CHECKING:
    from robot.motion import RobotMotion

class RobotNavigation:
    def __init__(
        self,
        robot_id,
        vision_system: VisionSystem,
        robot_motion: "RobotMotion",
        adjacency_list=None,
        seconds_per_grid_unit=1.0,
        on_node_arrived=None,
    ):
        self.robot_id = robot_id
        self.vision = vision_system
        self.motion = robot_motion
        self.adjacency_list = adjacency_list or {}
        self.seconds_per_grid_unit = seconds_per_grid_unit
        self.on_node_arrived = on_node_arrived
        self.current_route = None
        self.current_queue = []
        self.current_state = "IDLE"  # Track the robot's current state: "IDLE", "MOVING", or "QUEUED"
        self.current_direction = "NORTH"  # Track the robot's current direction: "NORTH", "EAST", "SOUTH", or "WEST"
        self.current_node = None

    def navigate_route(self, path):
        """Navigate the robot along the specified path."""
        self.current_route = path
        if not path:
            self.current_state = "IDLE"
            return

        self.current_state = "MOVING"
        self.current_queue = list(path)

        try:
            for index in range(len(path) - 1):
                current_state = path[index]
                next_state = path[index + 1]
                current_node = current_state[0]
                next_node = next_state[0]

                if current_node == next_node:
                    continue

                edge_data = self.adjacency_list.get(current_node, {}).get(next_node)
                if edge_data is None:
                    logging.error(
                        "Robot %s: no edge from %s to %s",
                        self.robot_id,
                        current_node,
                        next_node,
                    )
                    return

                target_direction = edge_data["direction"].upper()
                if target_direction != self.current_direction:
                    self.turn_to_direction(self.current_direction, target_direction)
                    self.current_direction = target_direction

                duration = edge_data["cost"] * self.seconds_per_grid_unit
                self.drive_forward_until_next_node(next_node, duration)
        finally:
            goal_node = path[-1][0]
            self.current_node = goal_node
            self.current_state = "IDLE"
            self.current_direction = "NORTH"
            self.current_route = None

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
                self.motion.turn_90_right(speed=0.3)
            elif delta == 2:
                self.motion.turn_90_right(speed=0.3)
                self.motion.turn_90_right(speed=0.3)
            elif delta == 3:
                self.motion.turn_90_left(speed=0.3)
        except ValueError as error:
            logging.error(
                "Robot %s encountered an unknown direction: %s",
                self.robot_id,
                error,
            )
        finally:
            current_direction = target_direction

    def notify_node_arrived(self, node, is_goal=False):
        """Notify the client that the robot has arrived at a node."""
        self.current_node = node
        if self.on_node_arrived is not None:
            self.on_node_arrived(node, is_goal=is_goal)
        
    def notify_delay(self, delay):
        """Notify the client that the robot has experienced a delay."""
        if self.on_node_arrived is not None:
            self.on_node_arrived(None, delay=delay)

    def drive_forward_until_intersection_node(self, next_node, duration_estimate):
        """Drive to the next node, then publish that arrival."""
        self.motion.move_forward(speed=0.2)
        start_time = time.time()
        delay_notified = False  # Track whether a delay has been notified
        CORRECTION_STEP = 0.02  # Step size for course correction

        TOLERANCE_MM = 20  # Tolerance for detecting side openings in millimeters
        
        try:
            while True:
                elapsed_time = time.time() - start_time
                left_distance, right_distance = self.motion.get_LiDAR_readings()
                difference = left_distance - right_distance

                if left_distance is not None and right_distance is not None:
                    if left_distance > 400 and right_distance > 400:  # Both sides are open, indicating an intersection
                        self.motion.robot.stop()
                        logging.info(f"Robot {self.robot_id} detected an intersection. Stopping.")
                        break
                    elif abs(difference) < TOLERANCE_MM:
                        if left_distance < right_distance:
                            self.motion.nodge_left(CORRECTION_STEP)
                        else:
                            self.motion.nodge_right(CORRECTION_STEP)
                    logging.info(f"Robot {self.robot_id} arrived at a node. Stopping.")
                    break

                elif elapsed_time > duration_estimate and delay_notified == False:
                    delay_duration = time.time() - start_time - duration_estimate
                    self.notify_delay(delay_duration)
                    delay_notified = True  # Mark that a delay has been notified


                time.sleep(0.05)  # Prevent busy waiting
        finally:
            self.motion.robot.stop()
            logging.info(
                f"Robot {self.robot_id} arrived at node {next_node}. Estimated duration: {duration_estimate:.2f} seconds."
            )
            self.current_state = "MOVING"  # Update state to E
            self.notify_node_arrived(next_node)



    def drive_forward_until_goaltag(self, duration_estimate):
        """Drive the robot forward until it detects the goal AprilTag."""
        self.motion.move_forward(speed=0.2)  # Move forward with a duration estimate
        try:
            while True:
                start_time = time.time()
                delay_notified = False  # Track whether a delay has been notified
                current_tag = self.vision.get_latest_tag()
                if current_tag is not None:
                    logging.info(f"Robot {self.robot_id} detected AprilTag ID: {current_tag}. Stopping.")
                    break
                elif time.time() - start_time > duration_estimate and delay_notified == False:
                    delay_duration = time.time() - start_time - duration_estimate
                    self.notify_delay(delay_duration)
                    delay_notified = True  # Mark that a delay has been notified
            time.sleep(0.1)  # Prevent busy waiting
        finally:
            self.motion.robot.stop()
            self.current_state = "IDLE"  # Update state to IDLE


    def check_for_side_distance(self):
        """Check for side distances using LiDAR readings."""
        self.LiDAR_left, self.LiDAR_right = self.motion.get_LiDAR_readings()
        return self.LiDAR_left, self.LiDAR_right

        

        


