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
ROUTE_DELAY_GRACE_SEC = 3
HOLD_COMMAND_RETRY_SEC = 2
HOLD_ACK_TIMEOUT_SEC = 10


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

        self.current_time = 0
        self._clock_origin = None
        self.robot_nodes = {}

        self.robot_states = {}
        self.robot_queues = defaultdict(deque)
        self.robot_started = {}
        self.robot_last_status_time = {}
        self.robot_route_steps = {}
        self.robot_route_lateness = {}
        self.robot_delay_notified = {}
        self.robot_recovery_state = {}
        self.robot_hold_requested_at = {}
        self.robot_hold_last_sent = {}
        self.robot_hold_request_ids = {}
        self.robot_hold_timeout_warned = set()



        if client is None:
            self.client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
                client_id="Server"
            )
        else:
            self.client = client

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def _refresh_current_time(self):
        if self._clock_origin is not None:
            # Elapsed monotonic time keeps wall-clock adjustments out of route schedules.
            self.current_time = int(time.monotonic() - self._clock_origin)
        return self.current_time

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code " + str(rc))

        client.subscribe("fleet/robots/+/status", qos=1)
        client.subscribe("fleet/robots/+/apriltag")
        client.subscribe("fleet/robots/+/error")
        client.subscribe("fleet/robots/+/command", qos=1)

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
                now = self._refresh_current_time()
                state = payload.get("state")
                node = payload.get("node")

                if self.robot_states.get(robot_id) == "ERROR":
                    print(f"Robot {robot_id} is in ERROR state. Ignoring status message.")
                    return

                if state == "HOLDING" and (
                    self.robot_recovery_state.get(robot_id) != "HOLD_REQUESTED"
                    or type(payload.get("hold_request_id")) is not int
                    or payload.get("hold_request_id") != self.robot_hold_request_ids.get(robot_id)
                ):
                    print(f"Ignoring unrequested or stale HOLDING status from robot {robot_id}.")
                    return

                route_step = payload.get("route_step")
                if state == "HOLDING" and not self._record_route_progress(
                    robot_id, node, route_step, now, state
                ):
                    print(f"Ignoring HOLDING status without valid route progress from robot {robot_id}.")
                    return

                self.robot_last_status_time[robot_id] = now
                
                if node is not None:
                    if robot_id not in self.fleet_manager.robot_home:
                        self.fleet_manager.register_robot(robot_id, node)
                    self.robot_nodes[robot_id] = node
                print(f"Received status from robot {robot_id}: state={state}, node={node}")
                if route_step is not None and state != "HOLDING":
                    self._record_route_progress(robot_id, node, route_step, now, state)

                self.robot_states[robot_id] = state
                if state == "HOLDING":
                    self._handle_robot_holding(robot_id, node)
                elif state == "IDLE":
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

    def _record_route_progress(self, robot_id, node, route_step, now, state):
        path = self.fleet_manager.active_paths.get(robot_id)
        # A route step indexes a timed path entry; wait entries count as steps too.
        if not path or type(route_step) is not int or not 0 <= route_step < len(path):
            print(f"Ignoring invalid route step from robot {robot_id}: {route_step}")
            return False
        if path[route_step][0] != node:
            print(
                f"Ignoring route step {route_step} from robot {robot_id}: "
                f"reported node {node}, expected {path[route_step][0]}"
            )
            return False

        previous_step = self.robot_route_steps.get(robot_id, 0)
        if route_step < previous_step:
            print(f"Ignoring stale route step from robot {robot_id}: {route_step}")
            return False

        self.robot_route_steps[robot_id] = route_step
        self.robot_route_lateness[robot_id] = max(0, now - path[route_step][2])
        if self.robot_route_lateness[robot_id] > ROUTE_DELAY_GRACE_SEC:
            if self.robot_delay_notified.get(robot_id) != route_step:
                print(
                    f"Robot {robot_id} is late at route step {route_step} by "
                    f"{self.robot_route_lateness[robot_id]} seconds."
                )
                self.robot_delay_notified[robot_id] = route_step
            if state == "MOVING":
                self._request_robot_hold(robot_id, now)
        return True

    def _request_robot_hold(self, robot_id, now):
        if self.robot_recovery_state.get(robot_id) == "HOLD_REQUESTED":
            return

        self.robot_recovery_state[robot_id] = "HOLD_REQUESTED"
        request_id = self.robot_hold_request_ids.get(robot_id, 0) + 1
        self.robot_hold_request_ids[robot_id] = request_id
        self.robot_hold_requested_at[robot_id] = now
        self.robot_hold_timeout_warned.discard(robot_id)
        self._publish_hold_command(robot_id, now)
        print(
            f"Requested robot {robot_id} to hold at its next node before replanning."
        )

    def _publish_hold_command(self, robot_id, now):
        self.robot_hold_last_sent[robot_id] = now
        topic = f"fleet/robots/{robot_id}/command"
        # Retries keep the same ID so the robot can ignore duplicate deliveries.
        self.client.publish(
            topic,
            json.dumps({
                "robot_id": robot_id,
                "action": "HOLD_AT_NEXT_NODE",
                "hold_request_id": self.robot_hold_request_ids[robot_id],
            }),
            qos=1,#QoS 1 ensures that the message is delivered at least once, which is important for critical commands like HOLD.
        )

    def _handle_robot_holding(self, robot_id, node):
        if self.robot_recovery_state.get(robot_id) != "HOLD_REQUESTED":
            print(f"Robot {robot_id} reported HOLDING without a pending hold request.")
            return
        if node is None:
            print(f"Robot {robot_id} reported HOLDING without a known node; cannot replan.")
            return

        old_path = self.fleet_manager.active_paths.get(robot_id)
        goal_node = old_path[-1][0] if old_path else None
        # Replace this robot's stale reservations while preserving every other route.
        self.fleet_manager.remove_reservations(robot_id)
        self.fleet_manager.active_paths.pop(robot_id, None)
        self.fleet_manager.register_robot(robot_id, node)
        self.robot_route_steps.pop(robot_id, None)
        self.robot_route_lateness.pop(robot_id, None)
        self.robot_recovery_state.pop(robot_id, None)
        self.robot_hold_requested_at.pop(robot_id, None)
        self.robot_hold_last_sent.pop(robot_id, None)
        self.robot_hold_timeout_warned.discard(robot_id)

        if goal_node is not None and goal_node != node:
            self.robot_queues[robot_id].appendleft(goal_node)

        self.robot_states[robot_id] = "IDLE"
        print(
            f"Robot {robot_id} is holding at {node}; replanning its route"
            + (f" to {goal_node}." if goal_node is not None else ".")
        )
        self.process_next_in_queue(robot_id)

    def _check_route_delays(self, now):
        for robot_id, recovery_state in list(self.robot_recovery_state.items()):
            if recovery_state != "HOLD_REQUESTED":
                continue
            if now - self.robot_hold_last_sent.get(robot_id, now) >= HOLD_COMMAND_RETRY_SEC:
                self._publish_hold_command(robot_id, now)
            if (
                now - self.robot_hold_requested_at.get(robot_id, now) >= HOLD_ACK_TIMEOUT_SEC
                and robot_id not in self.robot_hold_timeout_warned
            ):
                print(
                    f"Robot {robot_id} has not acknowledged the hold request; "
                    "it remains on its current route."
                )
                self.robot_hold_timeout_warned.add(robot_id)

        for robot_id, path in self.fleet_manager.active_paths.items():
            if self.robot_states.get(robot_id) != "MOVING":
                continue
            if self.robot_recovery_state.get(robot_id) == "HOLD_REQUESTED":
                continue
            next_step = self.robot_route_steps.get(robot_id, 0) + 1
            if next_step >= len(path):
                continue

            delay = now - path[next_step][2]
            if delay <= ROUTE_DELAY_GRACE_SEC:
                continue
            if self.robot_delay_notified.get(robot_id) == next_step:
                continue

            self.robot_route_lateness[robot_id] = delay
            self.robot_delay_notified[robot_id] = next_step
            print(
                f"Robot {robot_id} has not reported route step {next_step}; "
                f"it is late by {delay} seconds."
            )
            self._request_robot_hold(robot_id, now)


    def process_next_in_queue(self, robot_id):
        if not self.robot_started.get(robot_id):
            if self.robot_queues.get(robot_id):
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
        current_time = self._refresh_current_time()
        self.fleet_manager.cleanup_past_reservations(current_time)
        route = self.fleet_manager.request_route(robot_id, start_node, goal_node, current_time)
        if not route:
            print(f"No available route for robot {robot_id} from {start_node} to {goal_node}.")
            return False
        
        print(f"Assigned route to robot {robot_id}: {route}")
        
        self.robot_states[robot_id] = "MOVING"
        self.robot_route_steps[robot_id] = 0
        self.robot_route_lateness[robot_id] = 0
        self.robot_delay_notified.pop(robot_id, None)

        # Publish the route to the MQTT broker
        topic = f"fleet/robots/{robot_id}/route"
        route_json = json.dumps({
           "path": route,
        })

        self.robot_recovery_state.pop(robot_id, None)
        self.robot_hold_requested_at.pop(robot_id, None)
        self.robot_hold_last_sent.pop(robot_id, None)
        self.robot_hold_timeout_warned.discard(robot_id)
        self.client.publish(topic, route_json, qos=1)
        print(f"Published route to topic {topic}: {route_json}")
        return True
    
    def handle_robot_error(self, robot_id, last_known_node, expected_node):
        print(f"Error occurred for robot {robot_id}: last known node {last_known_node}, expected node {expected_node}")
        self.robot_states[robot_id] = "ERROR"
        if last_known_node is not None:
            self.robot_nodes[robot_id] = last_known_node
        self.robot_queues[robot_id].clear()
        self.robot_started[robot_id] = False
        self.robot_route_steps.pop(robot_id, None)
        self.robot_route_lateness.pop(robot_id, None)
        self.robot_delay_notified.pop(robot_id, None)
        self.robot_recovery_state.pop(robot_id, None)
        self.robot_hold_requested_at.pop(robot_id, None)
        self.robot_hold_last_sent.pop(robot_id, None)
        self.robot_hold_timeout_warned.discard(robot_id)
        print(f"Robot {robot_id} has encountered an error and is now marked as not started.")
        self.fleet_manager.block_nodes(robot_id, [last_known_node, expected_node])

    def reset_robot(self, robot_id, confirmed_node):
        with self.lock:
            self.fleet_manager.remove_reservations(robot_id)
            self.fleet_manager.active_paths.pop(robot_id, None)
            self.fleet_manager.register_robot(robot_id, confirmed_node)
            self.robot_nodes[robot_id] = confirmed_node
            self.robot_states[robot_id] = "IDLE"
            self.robot_recovery_state.pop(robot_id, None)
            self.robot_hold_requested_at.pop(robot_id, None)
            self.robot_hold_last_sent.pop(robot_id, None)
            self.robot_hold_timeout_warned.discard(robot_id)
            print(f"Robot {robot_id} has been reset to IDLE state and its queue cleared.")
        

    def start(self):
        self._clock_origin = time.monotonic()
        self.current_time = 0
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.loop_start()

        try:
            while True:
                time.sleep(1)
                with self.lock:
                    current_time = self._refresh_current_time()
                    self.fleet_manager.cleanup_past_reservations(current_time)
                    self._check_route_delays(current_time)
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