from pathfinding.graph import build_adjacency_matrix, save_adjacency_matrix_to_csv
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

    save_map_to_csv(warehouse_map, "warehouse_map.csv")
    save_adjacency_matrix_to_csv(*build_adjacency_matrix(warehouse_map), "adjacency_matrix.csv")

if __name__ == "__main__":
    main()