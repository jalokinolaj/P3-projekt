import os
import sys

# Add the root of the project (one level up from 'simulator') to Python path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)
from pathfinding.pathfinding import a_star_search, edge_key, permanent_node_key

class FleetManager:
    def __init__(self, adjacency_list, node_coordinates, step_cost, turn_cost, time_cost):
        self.adjacency_list = adjacency_list
        self.node_coordinates = node_coordinates
        self.step_cost = step_cost
        self.turn_cost = turn_cost
        self.time_cost = time_cost
        self.shared_timetable = {}  # Shared timetable for all robots
        self.active_paths = {}  # Store active paths for each robot
        self.robot_home = {}  # Store home nodes for each robot
    def register_robot(self, robot_id, node):
        self.robot_home.setdefault(robot_id, node)  # Set the home node for the robot
        self.shared_timetable[permanent_node_key(node)] = robot_id  # Reserve the home node in the timetable
        

    def request_route(self, robot_id, start_node, goal_node, start_time):
        old_path = self.active_paths.get(robot_id)
        self.remove_reservations(robot_id)  # Remove old reservations before calculating new path

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
            self.active_paths.pop(robot_id, None)
            self.shared_timetable[permanent_node_key(start_node)] = robot_id
            print(f"Robot {robot_id} could not find a path from {start_node} to {goal_node}.")
            return None

        self.active_paths[robot_id] = path
        self.update_timetable(robot_id, path)
        return path
    def expected_node(self, robot_id, current_time):
        path = self.active_paths.get(robot_id)
        if not path:
            return None  # No active path for this robot
        current = None

        for node, _, time in path:
            if time <= current_time:
                current = node
            else:
                break
        return current  # If current_time is beyond the last time in the path, return the last node

    def update_timetable(self, robot_id, path):
        # Update the shared timetable with the robot's path
        for node, _, time in path:
            self.shared_timetable[(node, time)] = robot_id  # Mark the edge as occupied by this robot
        for current_state, next_state in zip(path, path[1:]):
            current_node, _, current_time = current_state
            next_node, _, next_time = next_state
            if current_node == next_node:
                continue  # Skip if the robot stays in the same node

            for movement_time in range(current_time, next_time):
                self.shared_timetable[edge_key(current_node, next_node, movement_time)] = robot_id

        final_node, _, _ = path[-1]
        self.shared_timetable[permanent_node_key(final_node)] = robot_id

    def remove_reservations(self, robot_id):
        #Remove all reservations made by the robot from the shared timetable, before new path is calculated. To avoid conflicts with old path.
        for key, owner in list(self.shared_timetable.items()):
            if owner == robot_id:
                del self.shared_timetable[key]

    def reset_timetable(self):
        self.shared_timetable.clear()
        self.active_paths.clear()

    def cleanup_past_reservations(self, current_time):
        # Remove reservations that are in the past
        for key in list(self.shared_timetable.keys()):
            try:
                if isinstance(key, tuple) and len(key) == 2:
                    t = int(key[1])
                elif isinstance(key, str) and key.startswith("edge_"):
                    t = int(key.rsplit("_", 1)[1])
                else:
                    continue
            except (ValueError, TypeError, OverflowError, IndexError):
                continue
            if t < current_time:
                    del self.shared_timetable[key]

    def block_nodes(self, robot_id, nodes):
        # Block the specified nodes for the robot in the shared timetable
        self.remove_reservations(robot_id)  # Remove any existing reservations for this robot
        self.active_paths.pop(robot_id, None)  # Remove the active path for this robot
        for node in nodes:
            if node is not None:
                self.shared_timetable[permanent_node_key(node)] = robot_id  # Block the node for this robot

                