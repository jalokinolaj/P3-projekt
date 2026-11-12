import os
import sys

# Add the root of the project (one level up from 'simulator') to Python path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)
from pathfinding.pathfinding import a_star_search, edge_key

class FleetManager:
    def __init__(self, adjacency_list, node_coordinates, step_cost, turn_cost, time_cost):
        self.adjacency_list = adjacency_list
        self.node_coordinates = node_coordinates
        self.step_cost = step_cost
        self.turn_cost = turn_cost
        self.time_cost = time_cost
        self.shared_timetable = {}  # Shared timetable for all robots
        self.active_paths = {}  # Store active paths for each robot

    def request_route(self, robot_id, start_node, goal_node, start_time):
        
        path = a_star_search(
            self.adjacency_list,
            self.node_coordinates,
            start_node,
            goal_node,
            step_cost=self.step_cost,
            turn_cost=self.turn_cost,
            timetable=self.shared_timetable,
            start_time=start_time,
            time_cost=self.time_cost
        )

        if path is None:
            print(f"Robot {robot_id} could not find a path from {start_node} to {goal_node}.")
            return None

        self.active_paths[robot_id] = path
        self.update_timetable(robot_id, path)
        return path

    def update_timetable(self, robot_id, path):
        # Update the shared timetable with the robot's path
        for node, direction, time in path:
            self.shared_timetable[(node, time)] = robot_id  # Mark the edge as occupied by this robot
        for current_state, next_state in zip(path, path[1:]):
            current_node, _, current_time = current_state
            next_node, _, next_time = next_state
            if current_node == next_node:
                continue  # Skip if the robot stays in the same node

            for movement_time in range(current_time, next_time):
                self.shared_timetable[edge_key(current_node, next_node, movement_time)] = robot_id

    def reset_timetable(self):
        self.shared_timetable.clear()
        self.active_paths.clear()