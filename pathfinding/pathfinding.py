import heapq


def heuristic(current_node, goal_node, node_coordinates):

    node = current_node[0] if isinstance(current_node, tuple) else current_node  # Extract the node from the tuple
    goal = goal_node[0] if isinstance(goal_node, tuple) else goal_node  # Extract the goal node from the tuple
    r1, c1 = node_coordinates[node]
    r2, c2 = node_coordinates[goal]
    h = abs(r1 - r2) + abs(c1 - c2)  # Manhattan distance
    return h


def reconstruct_path(came_from, current_state):
    total_path = [current_state]
    while current_state in came_from:
        current_state = came_from[current_state]
        total_path.append(current_state)
    return total_path[::-1]  # Return reversed path

def pure_distance(path, adjacency_list):
    """Return the physical distance of a path without turn penalties."""
    distance = 0

    for current, next_node in zip(path, path[1:]):
        current_node = current[0] if isinstance(current, tuple) else current
        next_node = next_node[0] if isinstance(next_node, tuple) else next_node

        try:
            distance += adjacency_list[current_node][next_node]["cost"]
        except KeyError as error:
            raise ValueError(
                f"Path contains an invalid edge: {current_node!r} -> {next_node!r}"
            ) from error

    return distance

def a_star_search(adjacency_list, node_coordinates, start_node, goal_node, step_cost, turn_cost):
    start_state = (start_node, None)  # (node, direction)
    openSet = []
    
    h_start = heuristic(start_state, goal_node, node_coordinates)

    counter = 0  # Initialize a counter to break ties in the priority queue
    heapq.heappush(openSet, (h_start, counter, start_state))  # Push a tuple of (priority, counter, state)

    came_from = {}

    gScore = {node: float('inf') for node in adjacency_list}
    fScore = {node: float('inf') for node in adjacency_list}

    gScore[start_state] = 0
    fScore[start_state] = h_start

    open_set_states = {start_state}  # Keep track of states in the open set for quick membership testing

    while openSet:
        _, _, current_state = heapq.heappop(openSet)
        open_set_states.discard(current_state)

        node_id, current_direction = current_state

        if node_id == goal_node:
            return reconstruct_path(came_from, current_state)
        for neighbor_node, edge_data in adjacency_list[node_id].items():
            neighbor_dir = edge_data["direction"]
            edge_cost = edge_data["cost"]

            neighbor_state = (neighbor_node, neighbor_dir)


            tentative_gScore = gScore[current_state] + edge_cost  # Assuming uniform cost for each step

            if current_direction is not None and current_direction != neighbor_dir:
                tentative_gScore += turn_cost  # Add turn cost if changing direction

            if tentative_gScore < gScore.get(neighbor_state, float('inf')):
                came_from[neighbor_state] = current_state
                gScore[neighbor_state] = tentative_gScore
                fScore[neighbor_state] = gScore[neighbor_state] + heuristic(neighbor_state, goal_node, node_coordinates)

                if neighbor_state not in open_set_states:
                    counter += 1
                    heapq.heappush(openSet, (fScore[neighbor_state], counter, neighbor_state))
                    open_set_states.add(neighbor_state)

    return None  # Return None if no path is found