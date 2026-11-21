import os

from pathfinding.pathfinding import a_star_search
from pathfinding.graph import load_graph_data

DATA_PATH = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "data",
    "adjacency_list.json",
)


def test_two_robot_paths_reserve_separate_times():
    adjacency_list, node_coordinates = load_graph_data(DATA_PATH)
    shared_timetable = {}

    robot_1_path = a_star_search(
        adjacency_list,
        node_coordinates,
        start_node="S0",
        goal_node="D4",
        step_cost=1,
        turn_cost=1,
        timetable=shared_timetable,
        start_time=0,
        time_cost=1,
    )

    assert robot_1_path is not None
    assert robot_1_path[0][0] == "S0"
    assert robot_1_path[-1][0] == "D4"

    for node, _, time in robot_1_path:
        shared_timetable[(node, time)] = "Robot_1"

    robot_2_path = a_star_search(
        adjacency_list,
        node_coordinates,
        start_node="S1",
        goal_node="D3",
        step_cost=1,
        turn_cost=1,
        timetable=shared_timetable,
        start_time=0,
        time_cost=1,
    )

    assert robot_2_path is not None
    assert robot_2_path[0][0] == "S1"
    assert robot_2_path[-1][0] == "D3"