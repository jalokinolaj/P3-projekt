"""Draw what the lidar sees as a top-down picture (north is up).

Blue dots  = what the sensor measures.
Grey dots  = what the MAP says you should see from EXPECTED_NODE (optional).
If the blue dots sit on top of the grey ones, your walls match the map.

Press S in the plot window to save the current picture as a PNG.
Needs: pip install matplotlib
"""
import math

import matplotlib.pyplot as plt
import serial

from ld06_scan import read_scans, SERIAL_PORT, BAUD_RATE
from ld06_match import load_grid, node_cells, ray_distance, MOUNT_OFFSET

GRID_FILE = "warehouse_map.csv"
EXPECTED_NODE = None   # e.g. "S0" to overlay what the map expects; None = just show the scan
HEADING = 0            # compass direction the robot faces (0 = north)
RANGE_M = 3.0          # how far the picture reaches from the sensor, in metres


def scan_to_xy(scan, heading_deg):
    """Lidar points -> x (east) and y (north) in metres. Same angle rules as ld06_match.py."""
    xs, ys = [], []
    for angle, dist, _conf in scan:
        compass = math.radians((angle + heading_deg - MOUNT_OFFSET) % 360)
        xs.append(dist / 1000 * math.sin(compass))
        ys.append(dist / 1000 * math.cos(compass))
    return xs, ys


def expected_xy(grid, node):
    """What the map says the lidar should see from a node, as x/y in metres."""
    r, c = node_cells(grid)[node]
    xs, ys = [], []
    for deg in range(0, 360, 2):
        d = ray_distance(grid, r, c, deg) / 1000
        xs.append(d * math.sin(math.radians(deg)))
        ys.append(d * math.cos(math.radians(deg)))
    return xs, ys


def make_figure(grid=None, node=None, heading_deg=HEADING):
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")
    ax.set_xlim(-RANGE_M, RANGE_M)
    ax.set_ylim(-RANGE_M, RANGE_M)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("east  (m)")
    ax.set_ylabel("north (m)")
    for ring in range(1, int(RANGE_M) + 1):
        ax.add_patch(plt.Circle((0, 0), ring, fill=False, ls=":", color="gray"))
    for label, (x, y) in {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}.items():
        ax.text(x * (RANGE_M - 0.2), y * (RANGE_M - 0.2), label, ha="center", va="center",
                fontsize=14, color="gray")
    if grid is not None and node is not None:
        ex, ey = expected_xy(grid, node)
        ax.scatter(ex, ey, s=10, c="lightgray", label=f"map expects at {node}", zorder=1)
    measured = ax.scatter([], [], s=8, c="tab:blue", label="lidar", zorder=2)
    hx, hy = math.sin(math.radians(heading_deg)), math.cos(math.radians(heading_deg))
    ax.annotate("", xy=(hx * 0.3, hy * 0.3), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->", color="red", lw=2), zorder=3)
    ax.legend(loc="upper right")
    return fig, ax, measured


def update(ax, measured, scan, heading_deg, title):
    xs, ys = scan_to_xy(scan, heading_deg)
    measured.set_offsets(list(zip(xs, ys)) if xs else [[float("nan"), float("nan")]])
    ax.set_title(title)


def main():
    grid = load_grid(GRID_FILE) if EXPECTED_NODE else None
    ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    plt.ion()
    fig, ax, measured = make_figure(grid, EXPECTED_NODE, HEADING)
    save_requested = []
    fig.canvas.mpl_connect("key_press_event",
                           lambda e: save_requested.append(True) if e.key == "s" else None)
    try:
        for n, scan in enumerate(read_scans(ser), start=1):
            if not plt.fignum_exists(fig.number):
                break
            update(ax, measured, scan, HEADING,
                   f"scan {n}, {len(scan)} points  (red arrow = robot front, S = save)")
            if save_requested:
                save_requested.clear()
                name = f"scan_{n}.png"
                fig.savefig(name, dpi=150)
                print("Saved", name)
            plt.pause(0.001)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()