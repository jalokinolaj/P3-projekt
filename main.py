import json
from pathfinding.pathfinding import a_star_search, pure_distance
from pathfinding.graph import build_adjacency_matrix, save_adjacency_matrix_to_csv, save_adjacency_list_to_json, load_adjacency_list_from_json
from pathfinding.map import build_map, print_map, save_map_to_csv


def main():
    warehouse_map = build_map(
        width=48,
        height=48,
        border=3,
        shelf_width=2,
        shelf_length=8,
        cross_aisle=3,
        aisle_spacing=3,
        map_density=0.15,
        num_deliveries=5,
    )

    #save_map_to_csv(warehouse_map, "warehouse_map.csv")

    #node_names, adjacency_matrix, adjacency_list = build_adjacency_matrix(warehouse_map)

    #save_adjacency_matrix_to_csv(node_names, adjacency_matrix, "adjacency_matrix.csv")

    #save_adjacency_list_to_json(adjacency_list, "adjacency_list.json")

    #print("Warehouse map, adjacency matrix, and adjacency list have been saved to CSV and JSON files.")

    print("Starter server og indlæser kort...")
    
    # 1. Læs JSON-filen én gang ved opstart
    try:
        with open('adjacency_list.json', 'r', encoding='utf-8') as file:
            data = json.load(file)
            
        adjacency_list = data["adjacency_list"]
        node_coordinates = data["node_coordinates"]
        print("Kort indlæst succesfuldt!")
        
    except FileNotFoundError:
        print("Fejl: Kunne ikke finde 'adjacency_list.json'. Sørg for den ligger i mappen.")
        return

    # 2. Eksempel på at bruge dataen (f.eks. når en robot anmoder om en rute)
    start_node = "S1"
    goal_node = "D0"
    
    path = a_star_search(
        adjacency_list,
        node_coordinates,
        start_node,
        goal_node,
        step_cost=1,
        turn_cost=2,
    )
    
    print(f"Beregnet rute fra {start_node} til {goal_node}:", path)
    print(f"Rute længde: {len(path)} trin")
    print(f"Rute length without turn penalty: {pure_distance(path, adjacency_list)}")

    # 3. Her starter din servers primære løkke (f.eks. modtag besked fra JetBots osv.)
    # while True:
    #     afvent_robot_forespørgsel()
    

if __name__ == "__main__":
    main()