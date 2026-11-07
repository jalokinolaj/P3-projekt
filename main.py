import argparse
import json
from pathfinding.pathfinding import a_star_search, pure_distance
from pathfinding.graph import build_adjacency_matrix, save_adjacency_matrix_to_csv, save_adjacency_list_to_json, load_graph_data
from pathfinding.map import build_map, print_map, save_map_to_csv
def build_and_save_map(width, height, num_deliveries):
    warehouse_map = build_map(
        width=width,
        height=height,
        border=3,
        shelf_width=2,
        shelf_length=8,
        cross_aisle=3,
        aisle_spacing=3,
        map_density=0.15,
        num_deliveries=num_deliveries,
    )

    save_map_to_csv(warehouse_map, "warehouse_map.csv")

    node_names, adjacency_matrix, adjacency_list = build_adjacency_matrix(warehouse_map)

    save_adjacency_matrix_to_csv(node_names, adjacency_matrix, "adjacency_matrix.csv")

    save_adjacency_list_to_json(adjacency_list, "adjacency_list.json")

    print("Warehouse map, adjacency matrix, and adjacency list have been saved to CSV and JSON files.")

def run_a_star_test(start_node, goal_node):
    adjacency_list, node_coordinates = load_graph_data("adjacency_list.json")

    path = a_star_search(
        adjacency_list,
        node_coordinates,
        start_node,
        goal_node,
        step_cost=1,
        turn_cost=2,
        timetable={},
        start_time=0,
        time_cost=1
    )
    
    print(f"Found path from {start_node} to {goal_node}:", path)
    print(f"Path length: {len(path)} steps")
    print(f"Path length without turn penalty: {pure_distance(path, adjacency_list)} edge lengths")

def main():

    parser = argparse.ArgumentParser(description="Warehouse Map Builder and A* Pathfinding Tester")
    parser.add_argument('--build-map', action='store_true', help="Build and save a warehouse map.")
    parser.add_argument('--width', type=int, default=20, help="Width of the warehouse map.")
    parser.add_argument('--height', type=int, default=20, help="Height of the warehouse map.")
    parser.add_argument('--num-deliveries', type=int, default=5, help="Number of delivery points in the warehouse map.")
    parser.add_argument('--test', action='store_true', help="Run A* pathfinding test.")
    parser.add_argument('--start', type=str, help="Start node for A* pathfinding.")
    parser.add_argument('--goal', type=str, help="Goal node for A* pathfinding.")

    args = parser.parse_args()
    if args.build_map:
        build_and_save_map(args.width, args.height, args.num_deliveries)
    elif args.test:
        run_a_star_test(args.start, args.goal)
    else:
        print("No action specified. Use --build-map to build a map or --test to run A* pathfinding test.")
    

if __name__ == "__main__":
    main()