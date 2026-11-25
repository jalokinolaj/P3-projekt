from ld06_topo import (expected_view, check_node, best_matches,
                       DIRECTIONS, MOUNT_OFFSET)

GRID = [
    [1, 1, 1, 1, 1],
    [1, "I1", 0, "I2", 1],
    [1, 0, 1, 0, 1],
    [1, "I3", 0, "I4", 1],
    [1, 1, 1, 1, 1],
]


def fake_scan(view, heading):
    """Make a scan where each compass direction shows the given distance."""
    scan = []
    for name, compass in DIRECTIONS.items():
        center = (compass - heading + MOUNT_OFFSET) % 360
        for off in range(-8, 9, 2):
            scan.append(((center + off) % 360, view[name], 200))
    return scan


# Scan taken at I1, robot facing north
view_i1 = expected_view(GRID, 1, 1)
scan = fake_scan(view_i1, heading=0)

ok, err = check_node(GRID, scan, 0, "I1")
assert ok, f"should match I1, error {err}"

ok, err = check_node(GRID, scan, 0, "I4")
assert not ok, "should NOT match I4"

assert best_matches(GRID, scan, 0)[0][1] == "I1"

# Same place, robot turned to face east: the scan changes, the answer shouldn't
scan_east = fake_scan(view_i1, heading=90)
ok, err = check_node(GRID, scan_east, 90, "I1")
assert ok, "heading rotation broke the match"

print("All tests passed")
