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


def edge_key(from_node, to_node, time):
    return ("edge", from_node, to_node, time)


def permanent_node_key(node):
    return ("permanent_node", node)


def node_is_occupied(timetable, node, time):
    """Return whether a node is occupied at a particular time."""
    return (
        (node, time) in timetable
        or permanent_node_key(node) in timetable
    )


def a_star_search(adjacency_list, node_coordinates, start_node, goal_node, step_cost, turn_cost, timetable, start_time, time_cost):
    if node_is_occupied(timetable, start_node, start_time):
        return None
    if start_node != goal_node and permanent_node_key(goal_node) in timetable:
        return None

    start_state = (start_node, None, start_time)  # (node, direction, time)
    openSet = []
    
    h_start = heuristic(start_state, goal_node, node_coordinates)

    counter = 0  # Initialize a counter to break ties in the priority queue
    heapq.heappush(openSet, (h_start, counter, start_state))  # Push a tuple of (priority, counter, state)

    came_from = {}

    gScore = {start_state: 0}
    fScore = {start_state: h_start}

    open_set_states = {start_state}  # Keep track of states in the open set for quick membership testing

    while openSet:
        _, _, current_state = heapq.heappop(openSet)
        open_set_states.discard(current_state)

        node_id, current_direction, current_time = current_state

        if node_id == goal_node:
            return reconstruct_path(came_from, current_state)

        next_time_wait = current_time + time_cost
        wait_state = (node_id, current_direction, next_time_wait)

        if not node_is_occupied(timetable, node_id, next_time_wait):
            tentative_gScore = gScore[current_state] + time_cost  # Cost for waiting
            if tentative_gScore < gScore.get(wait_state, float('inf')):
                came_from[wait_state] = current_state
                gScore[wait_state] = tentative_gScore
                fScore[wait_state] = gScore[wait_state] + heuristic(wait_state, goal_node, node_coordinates)

                if wait_state not in open_set_states:
                    counter += 1
                    heapq.heappush(openSet, (fScore[wait_state], counter, wait_state))
                    open_set_states.add(wait_state)



        for neighbor_node, edge_data in adjacency_list[node_id].items():
            neighbor_dir = edge_data["direction"]
            edge_cost = edge_data["cost"]

            next_time_move = current_time + edge_cost
            neighbor_state = (neighbor_node, neighbor_dir)
            neighbor_state = (neighbor_node, neighbor_dir, next_time_move)

            edge_is_blocked = any(
                timetable_key in timetable
                for movement_time in range(current_time, next_time_move)
                for timetable_key in (
                    edge_key(node_id, neighbor_node, movement_time),
                    edge_key(neighbor_node, node_id, movement_time),
                )
            )
            if edge_is_blocked:
                continue  # Prevent same-direction and head-on edge collisions.

            if node_is_occupied(timetable, neighbor_node, next_time_move):
                continue  # Skip this neighbor if it's not available at the next time step
            tentative_gScore = gScore[current_state] + edge_cost  # Same cost for moving to the neighbor

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