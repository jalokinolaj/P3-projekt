

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

def test_finished_robot_does_not_log_not_started_without_pending_goals(server, capsys):
    boot(server, "robot1", "S0")
    for goal in ["D", "START_NODE_TRIGGER"]:
        send(server, "fleet/robots/robot1/apriltag", {"robot_id": "robot1", "goal_node": goal})
    send(server, "fleet/robots/robot1/status", {"robot_id": "robot1", "state": "IDLE", "node": "D"})
    send(server, "fleet/robots/robot1/status", {"robot_id": "robot1", "state": "IDLE", "node": "S0"})

    send(server, "fleet/robots/robot1/status", {"robot_id": "robot1", "state": "IDLE", "node": "S0"})

    assert "has not started yet" not in capsys.readouterr().out

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

def test_route_uses_server_monotonic_time(server, monkeypatch):
    import fleetmanager.server as server_module

    server._clock_origin = 100.0
    monkeypatch.setattr(server_module.time, "monotonic", lambda: 102.9)

    assert server.assign_route("robot1", "S0", "D")
    assert routes(server, "robot1")[0][0][2] == 2
    assert server.current_time == 2

def test_status_route_step_records_lateness(server, monkeypatch):
    import fleetmanager.server as server_module

    now = [100.0]
    server._clock_origin = 100.0
    monkeypatch.setattr(server_module.time, "monotonic", lambda: now[0])
    boot(server, "robot1", "S0")
    assert server.assign_route("robot1", "S0", "D")

    now[0] = 105.0
    send(server, "fleet/robots/robot1/status", {
        "robot_id": "robot1",
        "state": "MOVING",
        "node": "D",
        "route_step": 1,
    })

    assert server.robot_route_steps["robot1"] == 1
    assert server.robot_route_lateness["robot1"] == 4

def test_server_detects_missing_route_step_after_grace(server, monkeypatch):
    import fleetmanager.server as server_module

    now = [100.0]
    server._clock_origin = 100.0
    monkeypatch.setattr(server_module.time, "monotonic", lambda: now[0])
    boot(server, "robot1", "S0")
    assert server.assign_route("robot1", "S0", "D")

    now[0] = 105.0
    current_time = server._refresh_current_time()
    server._check_route_delays(current_time)

    assert server.robot_route_lateness["robot1"] == 4
    assert server.robot_delay_notified["robot1"] == 1

def test_late_robot_holds_then_replans_active_goal(server, monkeypatch):
    import fleetmanager.server as server_module

    now = [100.0]
    server._clock_origin = 100.0
    monkeypatch.setattr(server_module.time, "monotonic", lambda: now[0])
    boot(server, "robot1", "S0")
    for goal in ["B", "C", "START_NODE_TRIGGER"]:
        send(server, "fleet/robots/robot1/apriltag", {"robot_id": "robot1", "goal_node": goal})

    original_route = routes(server, "robot1")[0]
    assert original_route[-1][0] == "B"
    assert original_route[1][0] == "A"
    assert list(server.robot_queues["robot1"]) == ["C"]

    now[0] = 106.0
    server._check_route_delays(server._refresh_current_time())

    hold_commands = [
        payload
        for topic, payload in server.client.published_messages
        if topic == "fleet/robots/robot1/command"
    ]
    assert hold_commands == [{
        "robot_id": "robot1",
        "action": "HOLD_AT_NEXT_NODE",
        "hold_request_id": 1,
    }]

    now[0] = 108.0
    server._check_route_delays(server._refresh_current_time())
    retried_hold_commands = [
        payload
        for topic, payload in server.client.published_messages
        if topic == "fleet/robots/robot1/command"
    ]
    assert [payload["hold_request_id"] for payload in retried_hold_commands] == [1, 1]

    send(server, "fleet/robots/robot1/status", {
        "robot_id": "robot1",
        "state": "HOLDING",
        "node": "A",
        "route_step": 1,
        "hold_request_id": 0,
    })
    assert server.robot_states["robot1"] == "MOVING"
    assert server.robot_nodes["robot1"] == "S0"

    send(server, "fleet/robots/robot1/status", {
        "robot_id": "robot1",
        "state": "HOLDING",
        "node": "D",
        "route_step": 1,
        "hold_request_id": 1,
    })
    assert server.robot_states["robot1"] == "MOVING"
    assert server.robot_nodes["robot1"] == "S0"

    send(server, "fleet/robots/robot1/status", {
        "robot_id": "robot1",
        "state": "HOLDING",
        "node": "A",
        "route_step": 1,
        "hold_request_id": 1,
    })

    replanned_route = routes(server, "robot1")[1]
    assert replanned_route[0][0] == "A"
    assert replanned_route[-1][0] == "B"
    assert list(server.robot_queues["robot1"]) == ["C"]
    assert server.robot_states["robot1"] == "MOVING"
    assert "robot1" not in server.robot_recovery_state

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