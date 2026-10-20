from platform import node

import numpy as np
import pandas as pd
import json

direction_lookup = {
    (-1, 0): "north",
    (1, 0): "south",
    (0, -1): "west",
    (0, 1): "east"
}

def build_adjacency_matrix(grid):
    height = len(grid)
    width = len(grid[0])

    node_names = []
    node_positions = []
    for r in range(height):
        for c in range(width):
            cell = grid[r][c]
            if cell != 0 and cell != 1:  # If the cell is not an obstacle or open space
                node_names.append(cell)
                node_positions.append((r, c))
    num_nodes = len(node_names)


    adjacency_list = {name: {} for name in node_names}
    adjacency_matrix = np.full((num_nodes, num_nodes), np.inf)  # Initialize with infinity

    for i in range(num_nodes):
        adjacency_matrix[i][i] = 0  # Distance to itself is 0

    position_to_index = {pos: idx for idx, pos in enumerate(node_positions)}
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for i, (start_r, start_c) in enumerate(node_positions):
        node_name = node_names[i]
        for dr, dc in directions:

            direction_name = direction_lookup[(dr, dc)]

            r, c = start_r + dr, start_c + dc
            distance = 1

            while 0 <= r < height and 0 <= c < width:
                cell = grid[r][c]

                if cell == 1:
                    break

                if (r, c) in position_to_index:
                    j = position_to_index[(r, c)]
                    target_name = node_names[j]

                    adjacency_matrix[i][j] = distance
                    adjacency_matrix[j][i] = distance

                    adjacency_list[node_name][target_name] = {"cost": distance, "direction": direction_name}
                    break

                r += dr
                c += dc
                distance += 1   
    return node_names, adjacency_matrix, adjacency_list

        

def save_adjacency_matrix_to_csv(node_names, adjacency_matrix, filename):
    df = pd.DataFrame(adjacency_matrix, index=node_names, columns=node_names)
    df_readable = df.map(lambda x: 'inf' if np.isinf(x) else int(x))
    df_readable.to_csv(filename, index=True)

def save_adjacency_list_to_json(adjacency_list, filename):
    with open(filename, 'w') as f:
        json.dump(adjacency_list, f, indent=4)

def load_adjacency_list_from_json(filename):
    with open(filename, 'r') as f:
        adjacency_list = json.load(f)
    return adjacency_list