import os
import sys
import json
import time
import paho.mqtt.client as mqtt


current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(project_root)
from pathfinding.graph import load_graph_from_json
from fleetmanager.fleetmanager import FleetManager

fleet_manager = FleetManager()
import paho.mqtt.client as mqtt

MQTT_BROKER = "localhost"#Change this to server IP address if running on a different machine
MQTT_PORT = 1883


class FleetServer:
    def __init__(self):

        json_path=os.path.join(project_root, "data", "adjacency_list.json")
        self.adjacency_list, self.node_coordinates = load_graph_from_json(json_path)

        self.fleet_manager = FleetManager(
            self.adjacency_list,
            self.node_coordinates,
            step_cost=1,
            turn_cost=2,
            time_cost=1
        )

        self.current_time = 0  # Initialize current time

        self.client = mqtt.Client("Server")
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

        if "status" in topic:
            state = payload.get("state")
            node = payload.get("node")
            print(f"Received status from robot {robot_id}: state={state}, node={node}")
        elif "apriltag" in topic:
            tag_id = payload.get("tag_id")
            print(f"Received AprilTag detection from robot {robot_id}: tag_id={tag_id}")

    def assign_route(self, robot_id, start_node, goal_node):

        route = self.fleet_manager.request_route(robot_id, start_node, goal_node, self.current_time)
        if route:
            print(f"Assigned route to robot {robot_id}: {route}")

            topic = f"fleet/robots/{robot_id.lower()}/route"
            route_json = json.dumps(route)

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
                print(f"Current time: {self.current_time}")
        except KeyboardInterrupt:
            print("Shutting down...")
            self.client.loop_stop()
            self.client.disconnect()
if __name__ == "__main__":
    server = FleetServer()
    server.start()