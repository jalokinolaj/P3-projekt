"""Check a real lidar scan against the warehouse map (warehouse_map.csv).

For every node the map can tell us what the lidar SHOULD see looking straight
down each of the four aisles: the distance to the far end of the aisle.
Those far-end distances are what make nodes different from each other, because
all the shelves look alike. So we compare them with what the sensor measures.

Conventions (same as ld06_topo.py):
- 1 grid cell = 10 cm, the robot stands in the centre of its cell
- compass 0 = north = up the map (row - 1), 90 = east, clockwise
- lidar angle is clockwise from the robot's front
- OUTSIDE THE MAP COUNTS AS A WALL, so build physical walls along the map edge
"""
import csv
import math
import statistics

CELL_MM = 100
MOUNT_OFFSET = 0     # degrees: lidar angle that points to the robot's front
MAX_MM = 4000        # distances beyond this are clipped (real and simulated alike)
WINDOW_DEG = 3       # look +/- this many degrees around each aisle direction
DIRECTIONS = {"north": 0, "east": 90, "south": 180, "west": 270}


def load_grid(path):
    """Read warehouse_map.csv. Walls -> 1, open floor -> 0, labels stay text."""
    grid = []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            grid.append([int(c) if c in ("0", "1") else c for c in row])
    return grid


def crop_grid(grid, row_from, row_to, col_from, col_to):
    """Keep only part of the map (end values included), e.g. the part you built."""
    return [row[col_from:col_to + 1] for row in grid[row_from:row_to + 1]]


def node_cells(grid):
    """{'I3': (row, col), ...} for every labelled cell (S0, I3, D2, ...)."""
    return {cell: (r, c)
            for r, row in enumerate(grid)
            for c, cell in enumerate(row)
            if isinstance(cell, str)}


def _is_wall(grid, r, c):
    if r < 0 or c < 0 or r >= len(grid) or c >= len(grid[0]):
        return True
    return grid[r][c] == 1


def ray_distance(grid, r, c, compass_deg):
    """Distance (mm) from the centre of cell (r, c) to the first wall in a compass direction."""
    dr = -math.cos(math.radians(compass_deg))
    dc = math.sin(math.radians(compass_deg))
    pr, pc = r + 0.5, c + 0.5
    cr, cc = r, c
    step_r = 1 if dr > 0 else -1
    step_c = 1 if dc > 0 else -1
    inf = float("inf")
    t_r = ((cr + (1 if dr > 0 else 0)) - pr) / dr if abs(dr) > 1e-9 else inf
    t_c = ((cc + (1 if dc > 0 else 0)) - pc) / dc if abs(dc) > 1e-9 else inf
    d_r = abs(1 / dr) if abs(dr) > 1e-9 else inf
    d_c = abs(1 / dc) if abs(dc) > 1e-9 else inf
    limit = MAX_MM / CELL_MM
    while True:
        if t_r < t_c:
            t, cr, t_r = t_r, cr + step_r, t_r + d_r
        else:
            t, cc, t_c = t_c, cc + step_c, t_c + d_c
        if t >= limit:
            return MAX_MM
        if _is_wall(grid, cr, cc):
            return t * CELL_MM


def expected_view(grid, node):
    """What the lidar should see from a node: farthest distance (mm) down each aisle."""
    r, c = node_cells(grid)[node]
    return {name: max(ray_distance(grid, r, c, (compass + d) % 360)
                      for d in range(-WINDOW_DEG, WINDOW_DEG + 1))
            for name, compass in DIRECTIONS.items()}


def measured_view(scan, heading_deg):
    """Farthest lidar distance (mm) down each aisle. scan = [(angle, dist_mm, conf), ...].
    None where the sensor returned nothing in that direction."""
    view = {}
    for name, compass in DIRECTIONS.items():
        target = (compass - heading_deg + MOUNT_OFFSET) % 360
        dists = [min(d, MAX_MM) for a, d, _ in scan
                 if abs((a - target + 180) % 360 - 180) <= WINDOW_DEG]
        view[name] = max(dists) if dists else None
    return view


def view_error(measured, expected):
    """Average mismatch (mm) over the directions we could measure. Lower = better."""
    errs = [abs(measured[d] - expected[d]) for d in DIRECTIONS if measured[d] is not None]
    return sum(errs) / len(errs) if errs else None


def rank_nodes(grid, scan, heading_deg, candidates=None):
    """[(error_mm, node), ...] best first. candidates=None means every node."""
    measured = measured_view(scan, heading_deg)
    ranked = []
    for name in (candidates or list(node_cells(grid))):
        err = view_error(measured, expected_view(grid, name))
        if err is not None:
            ranked.append((err, name))
    return sorted(ranked)


def check_node(grid, scan, heading_deg, expected_node, tolerance_mm=80):
    """(matches, error_mm) for the node we think we are at."""
    err = view_error(measured_view(scan, heading_deg), expected_view(grid, expected_node))
    return err is not None and err <= tolerance_mm, err
