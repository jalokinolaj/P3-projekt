import json
import logging
import signal
import time
import paho.mqtt.client as mqtt
import threading

from robot.vision import VisionSystem
from config import *


class RobotClient:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.state = "IDLE"
        self.current_route = []
        self.current_node = "S0"
        self.stop_event = threading.Event()


        self.vision = VisionSystem()
        # Bruger VERSION2 for at undgå deprecation warning
        '''self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"Robot_{robot_id}"
        )'''
        self.client = mqtt.Client(client_id=f"Robot_{robot_id}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def run(self):
        self.connect()
        self.send_status(state="IDLE", node="S0")  # Initial status message

        try:
            while not self.stop_event.is_set():
                current_tag = self.vision.get_latest_tag()

                if current_tag is not None and current_tag != self.last_sent_tag:
                    node = TAG_TO_NODE.get(current_tag)


                    if node is not None:
                        self.send_apriltag_detection(tag_id=current_tag, node=node)
                        self.last_sent_tag = current_tag
                    else:
                        logging.warning(f"Robot {self.robot_id} detected unknown AprilTag ID: {current_tag}")
                        self.last_sent_tag = current_tag # Mark as sent to avoid re-sending

                elif current_tag is None:
                    self.last_sent_tag = None  # Reset if no tag is detected

                time.sleep(0.1)  # Prevent busy waiting
        finally:
            self.disconnect()
                

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            print(f"Robot {self.robot_id} connected successfully to MQTT broker!")
            client.subscribe(f"fleet/robots/{self.robot_id}/route")
            client.subscribe(f"fleet/robots/{self.robot_id}/command")
        else:
            print(f"Failed to connect, return code {reason_code}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode())
        except json.JSONDecodeError:
            logging.error(f"Robot {self.robot_id} received invalid JSON on topic {topic}: {msg.payload}")
            return
        if "route" in topic:
            path = payload.get("path")
            logging.info(f"\n[RECEIVED FROM SERVER] Robot {self.robot_id} modtog rute: {path}")
            if self.current_route:
                self.state = "MOVING"
                logging.info(f"Robot {self.robot_id} is currently moving. New route will be queued.")
        

        elif "command" in topic and payload.get("action") == "SHUTDOWN":
            logging.info(f"Robot {self.robot_id} is shutting down.")
            logging.info(f"\n[RECEIVED FROM SERVER] Robot {self.robot_id} modtog shutdown kommando.")
            self.stop_event.set()
            logging.info(f"Robot {self.robot_id} is shutting down.")

    def send_status(self, state, node):
        """Send the robot's status to the MQTT broker."""
        payload = {
            "robot_id": self.robot_id,
            "state": state,
            "node": node
        }
        self.client.publish(f"fleet/robots/{self.robot_id}/status", json.dumps(payload))
        logging.info(f"->Send status: state={state}, node={node}")

    def send_apriltag_detection(self, tag_id, node):
        payload = {
            "robot_id": self.robot_id,
            "tag_id": tag_id,
            "node": node
        }
        self.client.publish(f"fleet/robots/{self.robot_id}/apriltag", json.dumps(payload))
        print(f"-> Send AprilTag detection: Tag {tag_id} ({node})")

    def connect(self):
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.loop_start()
        self.vision.start()

    def disconnect(self):
        self.vision.stop()
        self.client.loop_stop()
        self.client.disconnect()

    def handle_sigterm(self, signal, frame):
        print(f"Robot {self.robot_id} received SIGTERM. Disconnecting...")
        self.stop_event.set()

if __name__ == "__main__":
    robot = RobotClient("robot_1")
    signal.signal(signal.SIGTERM, robot.handle_sigterm)
    robot.run()


