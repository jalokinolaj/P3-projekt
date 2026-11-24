import json
import queue
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import paho.mqtt.client as mqtt

BROKER = "localhost"
PORT = 1883
GOAL_DELAY_SEC = 50
CONNECT_TIMEOUT_SEC = 10
ROUTE_TIMEOUT_SEC = 10

ROBOT_SCENARIOS = {
    "robot_1": [
        {"start_tag": "S0", "goal_node": "D0"},
        {"start_tag": "D0", "goal_node": "S0"},
    ],
    "robot_2": [
        {"start_tag": "S1", "goal_node": "D1"},
        {"start_tag": "D1", "goal_node": "S1"},
    ],
}


def publish_json(client, topic: str, payload: dict) -> None:
    result = client.publish(topic, json.dumps(payload), qos=1)
    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        raise RuntimeError(f"Could not publish to {topic}: MQTT error {result.rc}")
    result.wait_for_publish(timeout=CONNECT_TIMEOUT_SEC)
    if not result.is_published():
        raise TimeoutError(f"Timed out publishing to {topic}")


def wait_for_route(route_messages: queue.Queue, robot_id: str, goal_node: str) -> None:
    try:
        _, raw_payload = route_messages.get(timeout=ROUTE_TIMEOUT_SEC)
    except queue.Empty as exc:
        raise TimeoutError(
            f"Timed out waiting for the server to assign robot {robot_id} "
            f"a route to {goal_node}"
        ) from exc

    try:
        payload = json.loads(raw_payload)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Server sent invalid route JSON for {robot_id}") from exc

    path = payload.get("path")
    if not isinstance(path, list) or not path:
        raise RuntimeError(f"Server sent an empty or invalid route for {robot_id}: {payload}")
    if path[-1][0] != goal_node:
        raise RuntimeError(
            f"Server assigned robot {robot_id} a route ending at {path[-1][0]}, "
            f"not {goal_node}"
        )


def run_simulated_robot(robot_id: str, routes: list[dict], delay_sec: float = GOAL_DELAY_SEC):
    if not routes:
        raise ValueError(f"No routes configured for {robot_id}")

    client = mqtt.Client(client_id=f"sim_{robot_id}")
    connected = threading.Event()
    subscribed = threading.Event()
    connection_errors = []
    subscription_mid = []
    route_messages = queue.Queue()
    route_topic = f"fleet/robots/{robot_id}/route"

    def on_connect(client, userdata, flags, reason_code, *extra):
        if reason_code != 0:
            connection_errors.append(f"MQTT connection failed with code {reason_code}")
            connected.set()
            return

        result, mid = client.subscribe(route_topic, qos=1)
        if result != mqtt.MQTT_ERR_SUCCESS:
            connection_errors.append(f"Could not subscribe to {route_topic}: MQTT error {result}")
        else:
            subscription_mid.append(mid)
        connected.set()

    def on_subscribe(client, userdata, mid, granted_qos, *extra):
        if subscription_mid and mid == subscription_mid[0]:
            subscribed.set()

    def on_message(client, userdata, message):
        if message.topic == route_topic:
            route_messages.put((message.topic, message.payload.decode("utf-8")))

    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    loop_started = False

    try:
        client.connect(BROKER, PORT, 60)
        client.loop_start()
        loop_started = True

        if not connected.wait(CONNECT_TIMEOUT_SEC):
            raise TimeoutError(f"Timed out connecting {robot_id} to {BROKER}:{PORT}")
        if connection_errors:
            raise ConnectionError(connection_errors[0])
        if not subscribed.wait(CONNECT_TIMEOUT_SEC):
            raise TimeoutError(f"Timed out subscribing {robot_id} to {route_topic}")

        initial_status = {
            "robot_id": robot_id,
            "state": "IDLE",
            "node": routes[0]["start_tag"],
        }
        publish_json(client, f"fleet/robots/{robot_id}/status", initial_status)

        expected_start = routes[0]["start_tag"]
        for route in routes:
            if route["start_tag"] != expected_start:
                raise ValueError(
                    f"Route for {robot_id} starts at {route['start_tag']}, "
                    f"but the previous route ends at {expected_start}"
                )
            publish_json(
                client,
                f"fleet/robots/{robot_id}/apriltag",
                {"robot_id": robot_id, "goal_node": route["goal_node"]},
            )
            expected_start = route["goal_node"]

        publish_json(
            client,
            f"fleet/robots/{robot_id}/apriltag",
            {"robot_id": robot_id, "goal_node": "START_NODE_TRIGGER"},
        )

        for route_num, route in enumerate(routes, start=1):
            wait_for_route(route_messages, robot_id, route["goal_node"])
            print(
                f"[{time.strftime('%H:%M:%S')}] {robot_id} | "
                f"Route {route_num} assigned: {route['start_tag']} -> {route['goal_node']}"
            )

            time.sleep(delay_sec)

            arrival_status = {
                "robot_id": robot_id,
                "state": "IDLE",
                "node": route["goal_node"],
            }
            publish_json(client, f"fleet/robots/{robot_id}/status", arrival_status)
            print(
                f"[{time.strftime('%H:%M:%S')}] {robot_id} | "
                f"Route {route_num} arrived at {route['goal_node']}"
            )
    finally:
        if loop_started:
            client.loop_stop()
            client.disconnect()


if __name__ == "__main__":
    with ThreadPoolExecutor(max_workers=max(1, len(ROBOT_SCENARIOS))) as executor:
        futures = [
            executor.submit(run_simulated_robot, robot_id, routes)
            for robot_id, routes in ROBOT_SCENARIOS.items()
        ]
        for future in futures:
            future.result()