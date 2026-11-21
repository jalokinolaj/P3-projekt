import json

import pytest
from fleetmanager.server import FleetServer

class FakeMQTTClient:
    def __init__(self):
        
        self.published_messages = []

    def publish(self, topic, payload=None, *a, **k):
        self.published_messages.append((topic, json.loads(payload)))
    def subscribe(self, *a, **k):
        pass  # Do nothing for subscribe in the fake client

class message:
    def __init__(self, topic, payload):
        self.topic = topic
        self.payload = json.dumps(payload).encode()  # Simulate the byte string payload

def edge(d): return {"direction": d, "cost": 1}
GRAPH = {
    "S0": {"A": edge("EAST"), "D": edge("SOUTH")},
    "A": {"S0": edge("WEST"), "B": edge("EAST"), "D": edge("SOUTH")},
    "B": {"A": edge("WEST"), "C": edge("EAST")},
    "C": {"B": edge("WEST"), "D": edge("EAST")},
    "D": {"A": edge("NORTH"), "C": edge("WEST"), "S0": edge("NORTH")},
}
COORDS = {
    "S0": (0, 0),
    "A": (0, 1),
    "B": (0, 2),
    "C": (0, 3),
    "D": (1, 1),
}

@pytest.fixture
def server(monkeypatch):
    import fleetmanager.server as server_module
    monkeypatch.setattr(server_module, "load_graph_data", lambda path: (GRAPH, COORDS))
    s = FleetServer(client=FakeMQTTClient())
    return s
def send(server, topic, payload):
    msg = message(topic, payload)
    server.on_message(server.client, None, msg)

def routes(server, robot_id):
    return [
        p["path"]
        for t, p in server.client.published_messages
        if t == f"fleet/robots/{robot_id}/route"
    ]