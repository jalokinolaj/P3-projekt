import os
import sys
import json
import time
import paho.mqtt.client as mqtt
import threading
from collections import defaultdict, deque

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(project_root)
from pathfinding.graph import load_graph_data
from fleetmanager.fleetmanager import FleetManager


MQTT_BROKER = "localhost"#Change this to server IP address if running on a different machine
MQTT_PORT = 1883

SCAN_TIMEOUT = 3.5


class FleetServer:
    def __init__(self, client=None, graph_path=None):

        json_path = graph_path or os.path.join(project_root,"data", "adjacency_list.json")
        self.adjacency_list, self.node_coordinates = load_graph_data(json_path)
        self.lock = threading.RLock()  # Lock for synchronizing access to shared resources
        self.fleet_manager = FleetManager(
            self.adjacency_list,
            self.node_coordinates,
            step_cost=1,
            turn_cost=2,
            time_cost=1
        )

        self.current_time = 0  # Initialize current time
        self.robot_nodes = {}

        self.robot_states = {}
        self.robot_queues = defaultdict(deque)
        self.robot_started = {}




        if client is None:
            self.client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
                client_id="Server"
            )
        else:
            self.client = client

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code " + str(rc))

        client.subscribe("fleet/robots/+/status")
        client.subscribe("fleet/robots/+/apriltag")
        client.subscribe("fleet/robots/+/error")
        client.subscribe("fleet/robots/+/command")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            raw_payload = msg.payload
            if isinstance(raw_payload, bytes):
                raw_payload = raw_payload.decode("utf-8")
            payload = json.loads(raw_payload)
        except (json.JSONDecodeError, UnicodeDecodeError):
            print(f"Received invalid JSON payload on topic {topic}: {msg.payload}")
            return

        robot_id = payload.get("robot_id")
        topic_robot_id = topic.split("/")[2]
        robot_id = robot_id or topic_robot_id
        with self.lock:
            if "status" in topic:
                
                state = payload.get("state")
                node = payload.get("node")

                if self.robot_states.get(robot_id) == "ERROR":
                    print(f"Robot {robot_id} is in ERROR state. Ignoring status message.")
                    return
                
                if node is not None:
                    if robot_id not in self.fleet_manager.robot_home:
                        self.fleet_manager.register_robot(robot_id, node)
                    self.robot_nodes[robot_id] = node
                print(f"Received status from robot {robot_id}: state={state}, node={node}")

                

                self.robot_states[robot_id] = state
                if state == "IDLE":
                    self.process_next_in_queue(robot_id)

            elif "apriltag" in topic:
                goal_node = payload.get("goal_node")
                print(f"Received AprilTag detection from robot {robot_id}: goal_node={goal_node}")

                if goal_node is None:
                    print(f"Received AprilTag detection from robot {robot_id} with no goal node.")
                    return
                if self.robot_states.get(robot_id) == "ERROR":
                    print(f"Robot {robot_id} is in ERROR state. Ignoring AprilTag detection.")
                    return

                if goal_node == "START_NODE_TRIGGER":
                    print(f"Robot {robot_id} is IDLE and received START_NODE_TRIGGER. Processing next in queue.")
                    self.robot_started[robot_id] = True
                    self.process_next_in_queue(robot_id)
                    return
                else:
                    self.robot_queues[robot_id].append(goal_node)
                    print(f"Robot {robot_id} queue: {goal_node}")

            elif "error" in topic:
                if self.robot_states.get(robot_id) == "ERROR":
                    print(f"Robot {robot_id} is in ERROR state. Ignoring error message.")
                    return
                self.handle_robot_error(
                    robot_id,
                    payload.get("last_known_node"),
                    payload.get("expected_node"),
                )
            elif "command" in topic:
                if payload.get("command") == "reset":
                    self.reset_robot(robot_id, payload.get("node"))



    def process_next_in_queue(self, robot_id):
        if not self.robot_started.get(robot_id):
            print(f"Robot {robot_id} has not started yet.")
            return
        if self.robot_states.get(robot_id) != "IDLE":
            print(f"Robot {robot_id} is not IDLE. Current state: {self.robot_states.get(robot_id)}")
            return
        start_node = self.robot_nodes.get(robot_id)
        if start_node is None:
            print(f"Cannot process queue for robot {robot_id} because its current node is unknown.")
            return
        queue = self.robot_queues[robot_id]
        home = self.fleet_manager.robot_home.get(robot_id)

        while queue:
            goal = queue.popleft()
            if goal == start_node:
                print(f"Robot {robot_id} is already at the goal node {goal}. Skipping.")
                continue
            if self.assign_route(robot_id, start_node, goal):
                print(f"Assigned route to robot {robot_id} from {start_node} to {goal}.")
                return
            else:
                print(f"Failed to assign route for robot {robot_id} from {start_node} to {goal}. Re-queuing the goal.")
                queue.appendleft(goal)  # Re-queue the goal
                return
        if home is not None and start_node != home:
            print(f"Robot {robot_id} has no more goals. Returning to home node {home}.")
            if self.assign_route(robot_id, start_node, home):
                print(f"Assigned route to robot {robot_id} from {start_node} to home node {home}.")
            return
        self.robot_started[robot_id] = False
        print(f"Robot {robot_id} has completed all tasks and returned home. Marking as not started.")

    def assign_route(self, robot_id, start_node, goal_node):
        self.fleet_manager.cleanup_past_reservations(self.current_time)
        route = self.fleet_manager.request_route(robot_id, start_node, goal_node, self.current_time)
        if not route:
            print(f"No available route for robot {robot_id} from {start_node} to {goal_node}.")
            return False
        
        print(f"Assigned route to robot {robot_id}: {route}")
        
        # Publish the route to the MQTT broker
        topic = f"fleet/robots/{robot_id}/route"
        route_json = json.dumps({
           "path": route,
        })

        self.client.publish(topic, route_json)
        self.robot_states[robot_id] = "MOVING"
        print(f"Published route to topic {topic}: {route_json}")
        return True
    
    def handle_robot_error(self, robot_id, last_known_node, expected_node):
        print(f"Error occurred for robot {robot_id}: last known node {last_known_node}, expected node {expected_node}")
        self.robot_states[robot_id] = "ERROR"
        if last_known_node is not None:
            self.robot_nodes[robot_id] = last_known_node
        self.robot_queues[robot_id].clear()
        self.robot_started[robot_id] = False
        print(f"Robot {robot_id} has encountered an error and is now marked as not started.")
        self.fleet_manager.block_nodes(robot_id, [last_known_node, expected_node])

    def reset_robot(self, robot_id, confirmed_node):
        with self.lock:
            self.fleet_manager.remove_reservations(robot_id)
            self.fleet_manager.active_paths.pop(robot_id, None)
            self.fleet_manager.register_robot(robot_id, confirmed_node)
            self.robot_nodes[robot_id] = confirmed_node
            self.robot_states[robot_id] = "IDLE"
            print(f"Robot {robot_id} has been reset to IDLE state and its queue cleared.")
        

    def start(self):
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.loop_start()

        try:
            while True:
                time.sleep(1)
                with self.lock:
                    self.current_time += 1
                    self.fleet_manager.cleanup_past_reservations(self.current_time)
                    for rid in list(self.robot_queues):
                        if self.robot_states.get(rid) == "IDLE":
                            self.process_next_in_queue(rid)
                print(f"Current time: {self.current_time}")
        except KeyboardInterrupt:
            print("Shutting down...")
            self.client.loop_stop()
            self.client.disconnect()
if __name__ == "__main__":
    server = FleetServer()
    server.start()