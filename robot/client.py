import json
import logging
import signal
import time
import paho.mqtt.client as mqtt
import threading

from robot.motion import RobotMotion
from robot.vision import VisionSystem
from robot.utils import get_voltage
from robot.navigation import RobotNavigation
from config import *


class RobotClient:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.state = "IDLE"
        self.current_route = []
        self.current_node = "S0"
        self.stop_event = threading.Event()
        self.last_sent_tag = None  # Track the last sent AprilTag to avoid duplicates
        self.last_sent_node = None  # Track the last sent node to avoid duplicates
        self.motion = RobotMotion(robot_id=robot_id)
        self.vision = VisionSystem()
        self.navigation = RobotNavigation(
            robot_id,
            self.vision,
            self.motion,
            on_node_arrived=self.on_node_arrived,
        )
        self.voltage = get_voltage()
        # Bruger VERSION2 for at undgå deprecation warning
        '''self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"Robot_{robot_id}"
        )'''
        self.client = mqtt.Client(client_id=f"Robot_{robot_id}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.route_lock = threading.Lock()  # Lock for synchronizing access to the current route
        self.busy = False  # Flag to indicate if the robot is currently busy navigating a route

    def run(self):
        self.connect()
        self.send_status(state="IDLE", node="S0")  # Initial status message

        try:
            while not self.stop_event.is_set():
                current_tag = self.vision.get_latest_tag()
                

                if current_tag is not None:
                    
                    if current_tag != self.last_sent_tag:  # Only send if the tag is different from the last sent
                        node = TAG_TO_NODE.get(str(current_tag))
                        print(f"Valid tag received: {current_tag} node: {node}")

                        if node is not None:
                            logging.info(f"Robot {self.robot_id} detected AprilTag ID: {current_tag}, sending to server.")
                            self.send_destination(node=node)

                        else:
                            logging.warning(f"Robot {self.robot_id} detected unknown AprilTag ID: {current_tag}")
                            
                        self.last_sent_tag = current_tag # Mark as sent to avoid re-sending

                else:
                    logging.info(f"Robot {self.robot_id} did not detect any AprilTag.")
                    pass

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
            logging.info(f"\n[RECEIVED FROM SERVER] Robot {self.robot_id} received route: {path}")

            if self.navigation.current_route is not None:
                logging.info(f"Robot {self.robot_id} is currently moving. New route will be queued.")
                return
            with self.route_lock:
                if self.busy:
                    logging.info(f"Robot {self.robot_id} is currently busy. New route will be queued.")
                    self.navigation.pending_routes.append(path)
                    return
                self.busy = True
            logging.info(f"Robot {self.robot_id} is starting navigation on the new route.")

            self.send_status(state="MOVING", node=self.current_node)
            threading.Thread(target=self.navigation.navigate_route, args=(path,)).start()
                
            
        

        elif "command" in topic and payload.get("action") == "SHUTDOWN":
            logging.info(f"Robot {self.robot_id} is shutting down.")
            logging.info(f"\n[RECEIVED FROM SERVER] Robot {self.robot_id} received shutdown command.")
            self.stop_event.set()
            logging.info(f"Robot {self.robot_id} is shutting down.")

    def run_navigation(self, path):
        try:
            self.navigation.navigate_route(path)
            while True:
                with self.route_lock:
                    if not self.navigation.pending_routes or self.current_state == "ERROR":
                        self.busy = False
                        return
                    path = self.navigation.pending_routes.pop(0)
                self.navigation.navigate_route(path)
        except Exception :
            logging.exception(f"Robot {self.robot_id} navigation crashed")
            self.navigation.handle_navigation_error(self.navigation.current_node)
            with self.route_lock:
                self.busy = False

    def on_error(self, last_known_node, expected_node):
        self.state = "ERROR"
        self.client.publish(f"fleet/robots/{self.robot_id}/error", json.dumps({
            "robot_id": self.robot_id,
            "last_known_node": last_known_node,
            "expected_node": expected_node
        })
        )
        self.send_status(state="ERROR", node=last_known_node)
    
    def on_node_arrived(self, node, is_goal=False, delay=None):
        if delay is not None:
            logging.warning(f"Robot {self.robot_id} experienced a delay of {delay:.2f} seconds at node {node}.")
            logging.info(f"Robot {self.robot_id} arrived at node {node}. Goal: {is_goal}")
            return
        if node is None:
            logging.warning(f"Robot {self.robot_id} arrived at an unknown node.")
            return
        self.current_node = node
        state = "IDLE" if is_goal else "MOVING"
        self.state = state
        self.send_status(state=state, node=node)


    def send_status(self, state, node):
        """Send the robot's status to the MQTT broker."""
        self.voltage = get_voltage()  # Update the voltage reading before sending status
        payload = {
            "robot_id": self.robot_id,
            "state": state,
            "node": node,
            "battery_voltage": round(self.voltage, 2)  # Include battery voltage in the status message
        }
        self.client.publish(f"fleet/robots/{self.robot_id}/status", json.dumps(payload))
        logging.info(f"->Send status: state={state}, node={node}, battery_voltage={round(self.voltage, 2)}")

    def send_destination(self, node):
        payload = {
            "robot_id": self.robot_id,
            "goal_node": node
        }
        self.client.publish(f"fleet/robots/{self.robot_id}/apriltag", json.dumps(payload))
        print(f"-> Send AprilTag detection: Tag ({node})")
        logging.info(f"Robot {self.robot_id} sent destination: {node}")

    

    def connect(self):
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.loop_start()
        self.vision.start()
        logging.info(f"Robot {self.robot_id} connected to the MQTT broker.")

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


