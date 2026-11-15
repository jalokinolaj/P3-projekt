import json
import time
import paho.mqtt.client as mqtt




MQTT_BROKER = "localhost"#Change this to server IP address if running on a different machine
MQTT_PORT = 1883
ROBOT_ID = "robot_1"  # Change this to the robot's unique ID

class RobotClient:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.client = mqtt.Client(f"Robot_{robot_id}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        print(f"Robot {self.robot_id} connected to MQTT broker with result code {rc}")
        client.subscribe(f"fleet/robots/{self.robot_id}/route")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode())
        except json.JSONDecodeError:
            print(f"Robot {self.robot_id} received invalid JSON payload on topic {topic}: {msg.payload}")
            return

        if "route" in topic:
            path = payload.get("path")
            directions = payload.get("directions", [])
            print(f"Robot {self.robot_id} received route: {path}")
            print(f"Robot {self.robot_id} received directions: {directions}")

    def send_status(self, state, node):
        payload = {
            "robot_id": self.robot_id,
            "state": state,
            "node": node
        }
        topic = f"fleet/robots/{self.robot_id}/status"
        self.client.publish(topic, json.dumps(payload))
        print(f"Robot {self.robot_id} sent status: {payload}")

    def send_apriltag_detection(self, tag_id):
        payload = {
            "robot_id": self.robot_id,
            "tag_id": tag_id
        }
        self.client.publish(f"fleet/robots/{self.robot_id}/apriltag", json.dumps(payload))
        print(f"Robot {self.robot_id} sent AprilTag destination: {payload}")

    def connect(self):
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.loop_start()

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()