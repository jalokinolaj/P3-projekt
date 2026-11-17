import os
import sys
import json
import time
import paho.mqtt.client as mqtt
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
    def __init__(self):

        json_path=os.path.join(project_root,"data", "adjacency_list.json")
        self.adjacency_list, self.node_coordinates = load_graph_data(json_path)

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




        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
            client_id="Server"
        )
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code " + str(rc))

        client.subscribe("fleet/robots/+/status")
        client.subscribe("fleet/robots/+/apriltag")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode())
        except json.JSONDecodeError:
            print(f"Received invalid JSON payload on topic {topic}: {msg.payload}")
            return

        robot_id = payload.get("robot_id")
        topic_robot_id = topic.split("/")[2]
        robot_id = robot_id or topic_robot_id

        if "status" in topic:
            state = payload.get("state")
            node = payload.get("node")
            if node is not None:
                self.robot_nodes[robot_id] = node
            print(f"Received status from robot {robot_id}: state={state}, node={node}")
            self.robot_states[robot_id] = state

            if state == "IDLE":
                print(f"Robot {robot_id} is IDLE. Processing next in queue.")
                self.process_next_in_queue(robot_id)
            else:
                print(f"Robot {robot_id} is {state}. Not processing queue.")


        elif "apriltag" in topic:
            goal_node = payload.get("goal_node")
            print(f"Received AprilTag detection from robot {robot_id}: goal_node={goal_node}")

            if goal_node is None:
                return
            current_state = self.robot_states.get(robot_id, "IDLE")
            start_node = self.robot_nodes.get(robot_id)

            if goal_node == "START_NODE_TRIGGER":
                print(f"Robot {robot_id} is IDLE and received START_NODE_TRIGGER. Processing next in queue.")
                self.process_next_in_queue(robot_id)
                return
                
            self.robot_queues[robot_id].append(goal_node)
            print(f"Robot {robot_id} is IDLE. Added goal_node {goal_node} to its queue.")



    def process_next_in_queue(self, robot_id):
        if self.robot_queues[robot_id]:
            self.fleet_manager.cleanup_past_reservations(self.current_time)
            
            next_goal_node = self.robot_queues[robot_id].popleft()
            start_node = self.robot_nodes.get(robot_id)
            if start_node is not None:
                self.assign_route(robot_id, start_node, next_goal_node)
            else:
                print(f"Cannot assign route for robot {robot_id} because its current node is unknown.")
        else:
            print(f"No queued goal nodes for robot {robot_id}.")

    def assign_route(self, robot_id, start_node, goal_node):

        route = self.fleet_manager.request_route(robot_id, start_node, goal_node, self.current_time)
        if route:
            print(f"Assigned route to robot {robot_id}: {route}")

            topic = f"fleet/robots/{robot_id.lower()}/route"
            route_json = json.dumps({
                "path": route,
                "directions": [direction for _, direction, _ in route if direction is not None]
            })

            self.client.publish(topic, route_json)
            print(f"Published route to topic {topic}: {route_json}")
        else:
            print(f"Could not assign route to robot {robot_id} from {start_node} to {goal_node}.")

    def start(self):
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.loop_start()

        try:
            while True:
                time.sleep(1)
                self.current_time += 1
                self.fleet_manager.cleanup_past_reservations(self.current_time)
                print(f"Current time: {self.current_time}")
        except KeyboardInterrupt:
            print("Shutting down...")
            self.client.loop_stop()
            self.client.disconnect()
if __name__ == "__main__":
    server = FleetServer()
    server.start()