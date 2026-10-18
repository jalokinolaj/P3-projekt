import numpy as np
import pandas as pd

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



    adjacency_matrix = np.full((num_nodes, num_nodes), np.inf)  # Initialize with infinity
    for i in range(num_nodes):
        adjacency_matrix[i][i] = 0  # Distance to itself is 0

    position_to_index = {pos: idx for idx, pos in enumerate(node_positions)}

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for i, (start_r, start_c) in enumerate(node_positions):
        for dr, dc in directions:
            r, c = start_r + dr, start_c + dc
            distance = 1

            while 0 <= r < height and 0 <= c < width:
                if grid[r][c] == 1:
                    break

                if (r, c) in position_to_index:
                    j = position_to_index[(r, c)]
                    adjacency_matrix[i][j] = distance
                    adjacency_matrix[j][i] = distance
                    break

                r += dr
                c += dc
                distance += 1   
    return node_names, adjacency_matrix

def save_adjacency_matrix_to_csv(node_names, adjacency_matrix, filename):
    df = pd.DataFrame(adjacency_matrix, index=node_names, columns=node_names)
    df_readable = df.map(lambda x: 'inf' if np.isinf(x) else int(x))
    df_readable.to_csv(filename, index=True)
