import os
import sys

# Tilføj roden af projektet (ét niveau op fra 'test') til Python path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)
# test_astar.py
import heapq
# Simuleret multi-agent test i Python
from pathfinding.pathfinding import a_star_search
from pathfinding.graph import load_graph_data

adjacency_list, node_coordinates = load_graph_data("../adjacency_list.json")

# Fælles timetable for hele flåden
shared_timetable = {}

# --- ROBOT 1 PLANLAGGER ---
robot_1_path = a_star_search(
    adjacency_list,
    node_coordinates,
    start_node="S0",
    goal_node="D4",
    step_cost=1,
    turn_cost=1,
    timetable=shared_timetable,  # Tomt i starten
    start_time=0,
    time_cost=1,
)

# Lås Robot 1's rute i det fældes timetable
for node, direction, time in robot_1_path:
  shared_timetable[(node, time)] = "Robot_1"

print("Robot 1 rute:", robot_1_path)


# --- ROBOT 2 PLANLAGGER (efter Robot 1 har booket) ---
robot_2_path = a_star_search(
    adjacency_list,
    node_coordinates,
    start_node="S1",
    goal_node="D3",
    step_cost=1,
    turn_cost=1,
    timetable=shared_timetable,  # Nu indeholder det Robot 1's tider!
    start_time=0,
    time_cost=1,
)

print("Robot 2 rute (skal undgå Robot 1):", robot_2_path)