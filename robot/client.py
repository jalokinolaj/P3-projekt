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
    def __init__(self, robot_id, start_node):
        self.robot_id = robot_id
        self.state = "IDLE"
        self.current_route = []
        self.current_node = start_node
        self.stop_event = threading.Event()
        self.last_sent_tag = None  # Track the last sent AprilTag to avoid duplicates
        self.last_sent_node = None  # Track the last sent node to avoid duplicates
        self.motion = RobotMotion(robot_id=robot_id)
        self.vision = VisionSystem()
        self.busy = False  # Flag to indicate if the robot is currently busy navigating a route
        self.navigation = RobotNavigation(
            robot_id,
            self.vision,
            self.motion,
            on_node_arrived=self.on_node_arrived,
            on_error=self.on_error
        )
        self.voltage = get_voltage()
        # Uses version 1 of the MQTT callback API for better compatibility with the RobotClient class. Since jetbot cant use V2.
        '''self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"Robot_{robot_id}"
        )'''
        self.client = mqtt.Client(client_id=f"Robot_{robot_id}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.route_lock = threading.Lock()  # Lock for synchronizing access to the current route
        # QoS 1 can redeliver commands, so track both seen and unacknowledged hold IDs.
        self.last_hold_request_id = 0
        self.pending_hold_request_id = None

    def run(self):
        self.connect()
        self.send_status(state="IDLE", node=self.current_node)  # Initial status message

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
        if reason_code == 0:# Successful connection
            print(f"Robot {self.robot_id} connected successfully to MQTT broker!")
            client.subscribe(f"fleet/robots/{self.robot_id}/route", qos=1)
            client.subscribe(f"fleet/robots/{self.robot_id}/command", qos=1)
        else:
            print(f"Failed to connect, return code {reason_code}")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except json.JSONDecodeError:
            logging.error(
                "Robot %s received invalid JSON on topic %s: %s",
                self.robot_id,
                msg.topic,
                msg.payload,
            )
            return
        if "route" in msg.topic:
            self._handle_route(payload.get("path"))
        elif "command" in msg.topic:
            self._handle_command(payload)

    def _handle_route(self, path):
        logging.info("Robot %s received route: %s", self.robot_id, path)
        with self.route_lock:
            if self.busy or self.navigation.current_state == "ERROR":
                logging.info("Robot %s is busy; ignoring the new route.", self.robot_id)
                state = "ERROR" if self.navigation.current_state == "ERROR" else "MOVING"
                self.send_status(state=state, node=self.current_node)
                return

            self.busy = True
            # A replacement route means the server accepted the previous hold acknowledgement.
            self.navigation.clear_hold_request()
            self.pending_hold_request_id = None

        self.last_sent_node = None
        self.navigation.current_route_step = 0
        self.send_status(state="MOVING", node=self.current_node, route_step=0)
        threading.Thread(target=self.run_navigation, args=(path,), daemon=True).start()

    def _handle_command(self, payload):
        action = payload.get("action")
        if action == "HOLD_AT_NEXT_NODE":
            self._handle_hold_command(payload.get("hold_request_id"))
        elif action in ("SHUTDOWN", "RESET"):
            logging.info("Robot %s received shutdown command.", self.robot_id)
            self.stop_event.set()

    def _handle_hold_command(self, request_id):
        if type(request_id) is not int:
            logging.error("Robot %s received a hold command without a valid request ID.", self.robot_id)
            return

        with self.route_lock:
            if request_id <= self.last_hold_request_id:
                # Ignore retries of a hold command already handled.
                return

            self.last_hold_request_id = request_id
            self.pending_hold_request_id = request_id
            self.navigation.request_hold_at_next_node()

            if not self.busy and self.navigation.current_state == "HOLDING":
                self._send_navigation_status(request_id)

    def run_navigation(self, path):
        try:
            self.navigation.navigate_route(path)
        except Exception:
            logging.exception(f"Robot {self.robot_id} navigation crashed")
            self.navigation.handle_navigation_error(self.navigation.current_node)
        finally:
            self.current_node = self.navigation.current_node
            with self.route_lock:
                self.busy = False
                # Keep the final status consistent with any hold command arriving now.
                self._send_navigation_status()

    def _send_navigation_status(self, hold_request_id=None):
        state = self.navigation.current_state
        if state not in ("IDLE", "HOLDING"):
            return

        if state == "HOLDING" and hold_request_id is None:
            hold_request_id = self.pending_hold_request_id

        self.send_status(
            state=state,
            node=self.navigation.current_node,
            route_step=self.navigation.current_route_step,
            hold_request_id=hold_request_id,
        )

    def on_error(self, last_known_node, expected_node):
        self.client.publish(f"fleet/robots/{self.robot_id}/error", json.dumps({
            "robot_id": self.robot_id,
            "last_known_node": last_known_node,
            "expected_node": expected_node
        }), qos=1)
        
        self.send_status(
            state="ERROR",
            node=last_known_node,
            route_step=self.navigation.current_route_step,
        )
    
    def on_node_arrived(self, node, is_goal=False, route_step=None):
        if node is None:
            logging.warning(f"Robot {self.robot_id} arrived at an unknown node.")
            return
        self.current_node = node
        if is_goal:
            return
        self.send_status(state="MOVING", node=node, route_step=route_step)


    def send_status(self, state, node, route_step=None, hold_request_id=None):
        """Send the robot's status to the MQTT broker."""
        self.voltage = get_voltage()  # Update the voltage reading before sending status
        payload = {
            "robot_id": self.robot_id,
            "state": state,
            "node": node,
            "battery_voltage": round(self.voltage, 2)  # Include battery voltage in the status message
        }
        if route_step is not None:
            payload["route_step"] = route_step
        if hold_request_id is not None:
            payload["hold_request_id"] = hold_request_id
        # QoS 1 asks MQTT to retry delivery if needed.
        self.client.publish(f"fleet/robots/{self.robot_id}/status", json.dumps(payload), qos=1)
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
    robot = RobotClient("robot_1", start_node="S0")
    signal.signal(signal.SIGTERM, robot.handle_sigterm)
    robot.run()


