import json
import time
import paho.mqtt.client as mqtt

MQTT_BROKER = "192.168.1.65"  # Din Windows IP
MQTT_PORT = 1883

class RobotClient:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        # Bruger VERSION2 for at undgå deprecation warning
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"Robot_{robot_id}"
        )
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            print(f"Robot {self.robot_id} connected successfully to MQTT broker!")
            client.subscribe(f"fleet/robots/{self.robot_id}/route")
        else:
            print(f"Failed to connect, return code {reason_code}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode())
        except json.JSONDecodeError:
            print(f"Robot {self.robot_id} received invalid JSON on topic {topic}: {msg.payload}")
            return

        if "route" in topic:
            path = payload.get("path")
            print(f"\n[RECEIVED FROM SERVER] Robot {self.robot_id} modtog rute: {path}")

    def send_status(self, state, node):
        """Send the robot's status to the MQTT broker."""
        payload = {
            "robot_id": self.robot_id,
            "state": state,
            "node": node
        }
        self.client.publish(f"fleet/robots/{self.robot_id}/status", json.dumps(payload))
        print(f"->Send status: state={state}, node={node}")

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

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()


if __name__ == "__main__":
    robot_id = "robot_1"
    print(f"Starting client for {robot_id}...")
    
    robot = RobotClient(robot_id)
    robot.connect()

    print("\n--- DUMMY SCANNER Activated ---")
    print("Chooes an action: 1. Send status, 2. Send AprilTag detection, or q to quit.")

    try:
        while True:
            valg = input("\nPlease choose an action (1, 2 or q): ").strip()
            
            if valg == 'q':
                break
            elif valg == '1':
                robot.send_status(state="IDLE", node="S0")
            elif valg == '2':
                robot.send_apriltag_detection(tag_id="D0", node="S0")
            else:
                print("Error: Invalid choice. Please select 1, 2, or q.")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nShutting down the robot client...")
    finally:
        robot.disconnect()
