import os
import sys

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, project_root)

from robot.navigation import RobotNavigation


class FakeMotion:
    def __init__(self):
        self.moves = []
        self.turns = []

    def move_forward(self, speed, duration):
        self.moves.append((speed, duration))

    def turn_90_left(self, speed):
        self.turns.append(("left", speed))

    def turn_90_right(self, speed):
        self.turns.append(("right", speed))


adjacency = {
    "S0": {"I0": {"cost": 5, "direction": "east"}},
    "I0": {"I1": {"cost": 5, "direction": "east"}},
}

fake = FakeMotion()
navigation = RobotNavigation(
    robot_id="test",
    robot_motion=fake,
    adjacency_list=adjacency,
    seconds_per_grid_unit=1.0,
)

navigation.navigate_route([
    ("S0", None, 0),
    ("I0", "east", 5),
    ("I1", "east", 10),
])

print(fake.moves)
print(fake.turns)