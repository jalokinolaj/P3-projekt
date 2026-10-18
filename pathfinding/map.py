import random

def generate_map(width, height, border, shelf_width, shelf_length, cross_aisle, aisle_spacing, map_density, num_deliveries):
    """ Generates a random map with given width and height, where each cell has a certain probability of being an obstacle."""
    grid = [[0 for x in range(width)] for y in range(height)]

#Cells count to get the number of obstacles to generate.
#Cell is 10 cm long and 10 cm wide.

    total_cells = width * height




    shelves_coords = []


    """calculated_aisle_spacing = shelf_area/(map_density * (shelf_length + cross_aisle))"""
    aisle_spacing = 3

    #Generate shelves in the grid with the specified spacing and dimensions.
    for y in range(border, height - border-shelf_length, shelf_length + aisle_spacing):
        for x in range(border, width - border-shelf_width, shelf_width + cross_aisle):
            for i in range(shelf_length):
                for j in range(shelf_width):
                    if y + i < height and x + j < width:
                        shelves_coords.append((x + j, y + i))
                        grid[y + i][x + j] = 1  # Mark the cell as an obstacle



    # Randomly place the delivery points in the grid, ensuring they are off the shelves.

    valid_delivery_spots = []
    for x, y in shelves_coords:
        if x>0 and grid[y][x-1] == 0:  # If the cell is not an obstacle
            valid_delivery_spots.append((x-2, y))


    valid_delivery_spots = list(set(valid_delivery_spots))  # Remove duplicates

    chosen_spot = random.sample(valid_delivery_spots, min(num_deliveries, len(valid_delivery_spots)))

    delivery_index = 0
    for x, y in chosen_spot:
        grid[y][x] = f"D{delivery_index}"  # Mark the cell as a delivery point
        delivery_index += 1


    grid[0][0] = 0  # Ensure the starting point is not an obstacle

    return grid

def place_intersections(grid, shelf_width, shelf_length, cross_aisle, aisle_spacing):
    height = len(grid)
    width = len(grid[0])
    

    start_y = aisle_spacing//2
    start_x = cross_aisle//2

    step_y = shelf_length + aisle_spacing
    step_x = shelf_width + cross_aisle

    intersections = []
    intersection_index = 0
    for r in range(start_y, height, step_y):
        for c in range(start_x, width, step_x):
            if grid[r][c] == 0:  # If the cell is not an obstacle
                intersections.append((r, c))
    if not intersections:
        return grid

    top_row = intersections[0][0]
    top_row_nodes = [pos for pos in intersections if pos[0] == top_row]

    

    top_left = min(top_row_nodes, key=lambda x: x[1])
    top_right = max(top_row_nodes, key=lambda x: x[1]) if len(top_row_nodes) > 1 else None
    

    for (r, c) in intersections:
        if (r, c) == top_left:
            grid[r][c] = "S0"
        elif (r, c) == top_right:
            grid[r][c] = "S1"
        else:
            grid[r][c] = f"I{intersection_index}"
            intersection_index += 1
    return grid

def trim_map(grid, border):
    #Find the last shelf in the grid and trim the map to that point.
    last_y=max(
        y for y, row in enumerate(grid)
        if 1 in row


    )

    last_x=max(
        x for row in grid
        for x, cell in enumerate(row)
        if cell == 1
    )

    # Trim the grid to the last shelf
    return [
        row[:last_x + border+1] 
        for row in grid[:last_y + border + 1]
        ]

def print_map(grid):
    """ Prints the map to the console."""
    for row in grid:
        print(' '.join(str(cell).ljust(3) for cell in row))
# Generate and print the map Width, Height, border, Shelf Width, Shelf Length, Map Density, and Number of Deliveries.
def build_map(width, height, border, shelf_width, shelf_length, cross_aisle, aisle_spacing, map_density, num_deliveries):
    """Builds the map by generating it, trimming it, placing intersections, and printing it."""
    map = generate_map(width, height, border, shelf_width, shelf_length, cross_aisle, aisle_spacing, map_density, num_deliveries)
    map = trim_map(map, border)
    map = place_intersections(map, shelf_width, shelf_length, cross_aisle, aisle_spacing)
    return map



if __name__ == "__main__":
    warehouse_map = build_map(48, 48, 3, 2, 8, 3, 3, 0.15, 5)
    print_map(warehouse_map)