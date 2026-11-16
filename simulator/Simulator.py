import os
import sys

# Add the root of the project (one level up from 'simulator') to Python path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

import time
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm

from fleetmanager.fleetmanager import FleetManager
from pathfinding.graph import load_graph_data

def get_robot_position_at_time(robot_path, node_coordinates, simulation_time):
    """Return a robots plot position at the given simulation time."""
    if simulation_time <= robot_path[0][2]:
        node = robot_path[0][0]
        row, column = node_coordinates[node]
        return column, row

    for current_state, next_state in zip(robot_path, robot_path[1:]):
        current_node, _, current_time = current_state
        next_node, _, next_time = next_state

        if simulation_time > next_time:
            continue

        start_row, start_column = node_coordinates[current_node]
        end_row, end_column = node_coordinates[next_node]
        duration = next_time - current_time

        if duration <= 0 or current_node == next_node:
            return end_column, end_row

        progress = (simulation_time - current_time) / duration
        progress = max(0.0, min(1.0, progress))
        column = start_column + (end_column - start_column) * progress
        row = start_row + (end_row - start_row) * progress
        return column, row

    final_row, final_column = node_coordinates[robot_path[-1][0]]
    return final_column, final_row

def run_simulation():
    # Load graph data
    graph_path = os.path.join(PROJECT_ROOT, "adjacency_list.json")
    adjacency_list, node_coordinates = load_graph_data(graph_path)
    
    # Stress scenario: Robot_1 uses I0 at t=5 on its shortest route.
    # Robot_2 would normally use I1 -> I0 -> I8 -> D1, but must take a
    # longer detour because I0 is reserved by Robot_1 at that time.
    robots_missions = [
        {"id": "Robot_1", "start_node": "S0", "goal_node": "D3"},
        {"id": "Robot_2", "start_node": "I1", "goal_node": "D1"},
    ]
    
    fleet_manager = FleetManager(
        adjacency_list,
        node_coordinates,
        step_cost=1,
        turn_cost=1,
        time_cost=1,
    )

    # Calculate and reserve paths in first-come, first-served order.
    for mission in robots_missions:
        robot_id = mission["id"]
        robot_path = fleet_manager.request_route(
            robot_id=robot_id,
            start_node=mission["start_node"],
            goal_node=mission["goal_node"],
            start_time=0,
        )
        
        if robot_path:
            print(f"{robot_id} path found:", robot_path)
    
    all_robot_paths = fleet_manager.active_paths

    # Check if any valid paths were found
    if not all_robot_paths:
        print("No robot paths found. Exiting simulation.")
        return

    # Size the figure from the map dimensions while keeping it usable for
    # both small and large maps.
    x_vals = [coords[1] for coords in node_coordinates.values()]
    y_vals = [coords[0] for coords in node_coordinates.values()]
    map_width = max(x_vals) - min(x_vals) + 1
    map_height = max(y_vals) - min(y_vals) + 1
    figure_scale = 0.8
    figsize = (
        max(6, min(20, map_width * figure_scale)),
        max(4, min(20, map_height * figure_scale)),
    )

    # 2. Start the visual simulation AFTER all paths are calculated
    plt.ion()  # Enable interactive mode
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_aspect("equal", adjustable="box")

    # Find the maximum time step across all generated paths
    max_time = max(max(p[2] for p in robot_path) for robot_path in all_robot_paths.values())

    # Generate unique colors for each robot
    colors = cm.rainbow(np.linspace(0, 1, len(all_robot_paths)))
    robot_colors = {robot_id: colors[i] for i, robot_id in enumerate(all_robot_paths.keys())}

    frame_interval = 1 / 60
    simulation_start = time.monotonic()
    simulation_time = 0.0

    # Draw static content once. Reusing these artists avoids flicker caused by
    # clearing and rebuilding the entire axes on every frame.
    ax.set_xlim(min(x_vals) - 0.5, max(x_vals) + 0.5)
    ax.set_ylim(min(y_vals) - 0.5, max(y_vals) + 0.5)
    ax.grid(True, alpha=0.25)
    ax.set_axisbelow(True)

    for n, coords in node_coordinates.items():
        y, x = coords
        ax.scatter(x, y, color="lightgray", s=300, zorder=1)
        ax.text(x, y + 0.3, n, fontsize=10, ha="center", zorder=2)

    robot_artists = {}
    for robot_id, robot_path in all_robot_paths.items():
        x, y = get_robot_position_at_time(
            robot_path,
            node_coordinates,
            simulation_time,
        )
        color = robot_colors[robot_id]
        robot_artist = ax.scatter(
            [x],
            [y],
            color=color,
            s=700,
            label=robot_id,
            edgecolors="black",
            linewidths=1.5,
            zorder=3,
        )
        robot_label = ax.text(
            x,
            y - 0.15,
            robot_id.replace("Robot_", "R"),
            color="white",
            fontweight="bold",
            ha="center",
            va="center",
            fontsize=8,
            zorder=4,
        )
        robot_artists[robot_id] = (robot_artist, robot_label)

    title = ax.set_title("Robot Simulation at Time Step: 0.0s")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.canvas.draw()

    # Run the animation in real time, updating only robot positions.
    while simulation_time <= max_time:
        title.set_text(f"Robot Simulation at Time Step: {simulation_time:.1f}s")

        for robot_id, robot_path in all_robot_paths.items():
            x, y = get_robot_position_at_time(
                robot_path,
                node_coordinates,
                simulation_time,
            )
            robot_artist, robot_label = robot_artists[robot_id]
            robot_artist.set_offsets([[x, y]])
            robot_label.set_position((x, y - 0.15))

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(frame_interval)
        simulation_time = time.monotonic() - simulation_start

    plt.ioff()  # Disable interactive mode
    plt.show()  # Show final plot window

if __name__ == "__main__":
    run_simulation()