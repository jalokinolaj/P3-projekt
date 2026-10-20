import random
 
 
def l_shape(w, h):
    """Cells (x, y) of an L-shaped shelf inside a w x h box (needs w, h >= 4).
    Two full-length arms along adjacent sides, random thickness and rotation."""
    t = random.randint(1, min(w, h) // 2 - 1)  # arm thickness
    cells = {(x, y) for x in range(w) for y in range(h) if x < t or y >= h - t}
    if random.random() < 0.5:  # mirror left/right
        cells = {(w - 1 - x, y) for x, y in cells}
    if random.random() < 0.5:  # mirror up/down
        cells = {(x, h - 1 - y) for x, y in cells}
    return cells
 
 
def layout(sizes, aisle):
    """Start position of each slot, aisle centre lines, and total length.
    Layout: aisle | slot | aisle | slot | ... | aisle"""
    starts, centers, pos = [], [aisle // 2], aisle
    for s in sizes:
        starts.append(pos)
        pos += s
        centers.append(pos + aisle // 2)
        pos += aisle
    return starts, centers, pos
 
 
def generate_map(num_cols, num_rows, aisle, shelf_min, shelf_max,
                 l_chance, num_deliveries):
    """Random warehouse. Each column/row gets a random size, so the map size,
    shelf sizes and shelf orientation (wide = horizontal, tall = vertical)
    are all random. Some shelves are L-shaped."""
    col_w = [random.randint(shelf_min, shelf_max) for _ in range(num_cols)]
    row_h = [random.randint(shelf_min, shelf_max) for _ in range(num_rows)]
 
    col_x, aisle_x, width = layout(col_w, aisle)
    row_y, aisle_y, height = layout(row_h, aisle)
 
    grid = [[0] * width for _ in range(height)]
 
    for cx, w in zip(col_x, col_w):
        for ry, h in zip(row_y, row_h):
            if min(w, h) >= 4 and random.random() < l_chance:
                cells = l_shape(w, h)
            else:
                cells = {(x, y) for x in range(w) for y in range(h)}
            for x, y in cells:
                grid[ry + y][cx + x] = 1
 
    # Delivery points: free cells next to a shelf, never on an intersection.
    nodes = {(r, c) for r in aisle_y for c in aisle_x}
    spots = []
    for y in range(height):
        for x in range(width):
            if grid[y][x] != 0 or (y, x) in nodes:
                continue
            for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < height and 0 <= nx < width and grid[ny][nx] == 1:
                    spots.append((y, x))
                    break
 
    for i, (y, x) in enumerate(random.sample(spots, min(num_deliveries, len(spots)))):
        grid[y][x] = f"D{i}"
 
    return grid, aisle_x, aisle_y
 
 
def place_intersections(grid, aisle_x, aisle_y):
    """Label every crossing of an aisle centre line. Top-left = S0, top-right = S1."""
    index = 0
    for r in aisle_y:
        for c in aisle_x:
            if (r, c) == (aisle_y[0], aisle_x[0]):
                grid[r][c] = "S0"
            elif (r, c) == (aisle_y[0], aisle_x[-1]):
                grid[r][c] = "S1"
            else:
                grid[r][c] = f"I{index}"
                index += 1
    return grid
 
 
def print_map(grid):
    for row in grid:
        print(' '.join(str(cell).ljust(3) for cell in row))
 
 
def build_map(cols=(3, 6), rows=(2, 4), aisle=3, shelf_min=2, shelf_max=10,
              l_chance=0.3, num_deliveries=5):
    num_cols = random.randint(*cols)
    num_rows = random.randint(*rows)
    grid, aisle_x, aisle_y = generate_map(num_cols, num_rows, aisle,
                                          shelf_min, shelf_max,
                                          l_chance, num_deliveries)
    return place_intersections(grid, aisle_x, aisle_y)
 
 
if __name__ == "__main__":
    warehouse = build_map()
    print(len(warehouse[0]), "x", len(warehouse))
    print_map(warehouse)