"""No-sensor test: simulate lidar scans on the real map and check the matcher finds the right node."""
import math
import random
from ld06_match import load_grid, node_cells, rank_nodes, check_node, _is_wall, MOUNT_OFFSET

grid = load_grid("warehouse_map.csv")
nodes = node_cells(grid)
random.seed(1)


def fake_scan(node, heading, off_mm=(0, 0), heading_err=0, noise=15):
    """Shoot rays from slightly off the cell centre, add noise. Like a real sensor would see."""
    r, c = nodes[node]
    pr, pc = r + 0.5 + off_mm[0] / 100, c + 0.5 + off_mm[1] / 100
    scan = []
    for k in range(450):
        a = k * 360 / 450
        comp = math.radians((a + heading - MOUNT_OFFSET + heading_err) % 360)
        d = 0
        while d < 12000:
            d += 4
            if _is_wall(grid, math.floor(pr - math.cos(comp) * d / 100),
                        math.floor(pc + math.sin(comp) * d / 100)):
                break
        scan.append((a, d + random.gauss(0, noise), 200))
    return scan


for label, kw in [("perfectly placed", {}),
                  ("3 cm off-centre, 2 deg off", dict(off_mm=(30, -30), heading_err=2)),
                  ("5 cm off-centre, 4 deg off", dict(off_mm=(50, 40), heading_err=4))]:
    right = 0
    for n in nodes:
        right += rank_nodes(grid, fake_scan(n, 0, **kw), 0)[0][1] == n
    print(f"{label}: right node found for {right} of {len(nodes)} nodes")

scan = fake_scan("I20", 0)
print("I20 scan checked against I20:", check_node(grid, scan, 0, "I20"))
print("I20 scan checked against I21:", check_node(grid, scan, 0, "I21"))
print("I20 scan checked against I9 :", check_node(grid, scan, 0, "I9"))
