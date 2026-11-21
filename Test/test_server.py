

from configtest import *


def boot(server, robot, node):
    send(server, f"fleet/robots/{robot}/status", {"robot_id": robot, "state": "IDLE", "node": node})

def test_no_route_before_start(server):
    boot(server, "robot1", "S0")
    send(server, "fleet/robots/robot1/apriltag", {"robot_id": "robot1", "goal_node": "D"})
    assert routes(server, "robot1") == []

def test_start_send_first_route(server):
    boot(server, "robot1", "S0")
    send(server, "fleet/robots/robot1/apriltag", {"robot_id": "robot1", "goal_node": "D"})
    send(server, "fleet/robots/robot1/apriltag", {"robot_id": "robot1", "goal_node": "START_NODE_TRIGGER"})
    assert len(routes(server, "robot1")) == 1
    assert routes(server, "robot1")[0] == [["S0", None, 0], ["D", "SOUTH", 1]]
    assert server.robot_states["robot1"] == "MOVING"

def test_idle_send_next_goal(server):
    boot(server, "robot1", "S0")
    for g in ["D", "START_NODE_TRIGGER", "B", "START_NODE_TRIGGER"]:
        send(server, "fleet/robots/robot1/apriltag", {"robot_id": "robot1", "goal_node": g})
    send(server, "fleet/robots/robot1/status", {"robot_id": "robot1", "state": "IDLE", "node": "D"})
    assert len(routes(server, "robot1")) == 2
    assert routes(server, "robot1")[1] == [["D", None, 0], ["C", "WEST", 1], ["B", "WEST", 2]]

def test_home_and_empty_queue(server):
    boot(server, "robot1", "S0")
    for g in ["D", "START_NODE_TRIGGER"]:
        send(server, "fleet/robots/robot1/apriltag", {"robot_id": "robot1", "goal_node": g})
    send(server, "fleet/robots/robot1/status", {"robot_id": "robot1", "state": "IDLE", "node": "D"})
    send(server, "fleet/robots/robot1/status", {"robot_id": "robot1", "state": "IDLE", "node": "S0"})
    assert server.robot_started["robot1"] == False

def test_double_idle_does_not_send_2_routes(server):
    boot(server, "robot1", "S0")
    for g in ["D", "START_NODE_TRIGGER"]:
        send(server, "fleet/robots/robot1/apriltag", {"robot_id": "robot1", "goal_node": g})
    n = len(routes(server, "robot1"))
    send(server, "fleet/robots/robot1/status", {"robot_id": "robot1", "state": "MOVING", "node": "S0"})
    assert len(routes(server, "robot1")) == n

def test_error_blocks_and_clears(server):
    boot(server, "robot", "S0")
    for g in ["D", "START_NODE_TRIGGER"]:
        send(server, "fleet/robots/robot/apriltag", {"robot_id": "robot", "goal_node": g})
    send(server, "fleet/robots/robot/error", {"robot_id": "robot", "last_known_node": "A", "expected_node": "D"})
    assert server.robot_states["robot"] == "ERROR"
    assert not server.robot_queues["robot"]
    assert server.robot_started["robot"] == False

def test_failed_planning_requeues_goal(server):
    boot(server, "robot1", "S0")
    boot(server, "robot2", "A")

    server.fleet_manager.block_nodes("robot2", ["D"])
    for g in ["D", "START_NODE_TRIGGER"]:
        send(server, "fleet/robots/robot1/apriltag", {"robot_id": "robot1", "goal_node": g})
    assert list(server.robot_queues["robot1"]) == ["D"]
    assert routes(server, "robot1") == []

def test_two_robots_have_independet_queues(server):
    boot(server, "robot1", "S0")
    boot(server, "robot2", "A")
    send(server, "fleet/robots/robot1/apriltag", {"robot_id": "robot1", "goal_node": "D"})
    assert list(server.robot_queues["robot1"]) == ["D"]
    assert list(server.robot_queues["robot2"]) == []

def test_invalid_json_not_crashing(server):
    class InvalidMessage:
        topic = "fleet/robots/robot1/status"
        payload = "{invalid_json}"
    server.on_message(None, None, InvalidMessage())
    assert True  # If we reach here, the test passes

def test_error_state_is_not_overwritten_by_status(server):
    boot(server, "robot1", "S0")
    send(server, "fleet/robots/robot1/error", {"robot_id": "robot1", "last_known_node": "A", "expected_node": "D"})
    assert server.robot_states["robot1"] == "ERROR"
    send(server, "fleet/robots/robot1/status", {"robot_id": "robot1", "state": "IDLE", "node": "A"})
    assert server.robot_states["robot1"] == "ERROR"

def test_error_robot_position_not_changed_by_status(server):
    boot(server, "r1", "S1")
    send(server, "fleet/robots/r1/error", {"robot_id": "r1", "last_known_node": "A", "expected_node": "B"})
    send(server, "fleet/robots/r1/status", {"robot_id": "r1", "state": "MOVING", "node": "S2"})
    assert server.robot_nodes["r1"] == "A"