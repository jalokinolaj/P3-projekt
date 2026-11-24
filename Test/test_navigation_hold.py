import os
import sys

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, project_root)

from robot.navigation import RobotNavigation


class FakeMotion:
    def turn_90_left(self):
        pass

    def turn_90_right(self):
        pass


class FastNavigation(RobotNavigation):
    def drive_forward_until_intersection_node(self, next_node, duration_estimate):
        return True

    def drive_forward_until_goaltag(self, duration_estimate):
        return True


def test_hold_request_stops_at_next_node_and_accepts_replacement_route():
    arrived = []
    navigation = None

    def on_node_arrived(node, is_goal=False, route_step=None):
        arrived.append((node, route_step))
        if node == "A":
            navigation.request_hold_at_next_node()

    navigation = FastNavigation(
        robot_id="test",
        vision_system=object(),
        robot_motion=FakeMotion(),
        on_node_arrived=on_node_arrived,
    )

    navigation.navigate_route([
        ("S0", None, 0),
        ("A", "EAST", 1),
        ("B", "EAST", 2),
    ])

    assert navigation.current_state == "HOLDING"
    assert navigation.current_node == "A"
    assert navigation.current_route_step == 1
    assert arrived == [("A", 1)]

    navigation.clear_hold_request()
    navigation.navigate_route([
        ("A", None, 6),
        ("B", "EAST", 7),
    ])

    assert navigation.current_state == "IDLE"
    assert navigation.current_node == "B"
    assert arrived[-1] == ("B", 1)
