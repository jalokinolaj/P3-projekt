import struct
import math
import threading
import time

# Linux: '/dev/ttyUSB0' | macOS: '/dev/tty.usbserial-0001' | Windows: 'COM12'
#Settings for sensoren
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 230400
ANGLE_OFFSET = 0.0 #skal ændres DA HVOR NUL IKKE SIDDER RIGTIG
MIN_CONFIDENCE = 150 # IGNORE ALT DER HAR EN CONFIDENCE PÅ UNDER 100
DEAD_ZONE = [] #ADD HER HVIS VI VIL HAVE DEAD ZONES 

#grid predections
CELL_MM = 100 #LIGE NU HAR VI AT EN NODE ER 10CM ÆNDRE DETTE HVIS NODE STØRRELSE ÆNDRES
N_SECTORS = 24 # KAN ÆNDRES DELER 360 TIL 24
MAX_COMPARE_MM = 2000 # BRUGES TIL AT SAMMENLIGNE TING 
TOLERANCE_BASE_MM = 40         # allowed mismatch per sector: this much ...
TOLERANCE_RELATIVE = 0.2       # ... plus this fraction of the expected distance
MATCH_SCORE = 1.0              # average score at or below this = "same place"
MIN_KNOWN_FRACTION = 0.75      # need lidar data in this fraction of the sectors to judge
BORDER_IS_WALL = True          # True if the edge of the grid is a real wall

HEADINGS = ["north", "east", "south", "west"]      # clockwise order
STEP = {"north": (-1, 0), "east": (0, 1), "south": (1, 0), "west": (0, -1)}
SECTOR_DEG = 360.0 / N_SECTORS


# ========================================================= 2. LD06 LIDAR READING =
def in_sector(angle, start, end):
    """True if angle is between start and end (degrees). start > end means it wraps through 0."""
    if start <= end:
        return start <= angle <= end
    return angle >= start or angle <= end


def in_deadzone(angle):
    return any(in_sector(angle, start, end) for start, end in DEAD_ZONES)


def ld06_crc_ok(payload):
    """CRC-8 (poly 0x4D, start 0) over the header 0x54 0x2C plus payload[:-1]."""
    crc = 0
    for byte in b"\x54\x2c" + payload[:-1]:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc == payload[-1]


def ld06_points(ser):
    """Yield (angle_deg, distance_mm, confidence) for every valid point from the serial port."""
    while True:
        if ser.read(1) == b"\x54" and ser.read(1) == b"\x2c":
            payload = ser.read(45)
            if len(payload) < 45 or not ld06_crc_ok(payload):
                continue
            _, start_angle = struct.unpack("<HH", payload[0:4])
            end_angle, _, _ = struct.unpack("<HHB", payload[40:45])
            start_deg = start_angle / 100.0
            end_deg = end_angle / 100.0
            step = ((end_deg - start_deg) % 360.0) / 11.0
            for i in range(12):
                offset = 4 + i * 3
                dist_mm, confidence = struct.unpack("<HB", payload[offset:offset + 3])
                if dist_mm > 0:
                    yield (start_deg + i * step) % 360.0, dist_mm, confidence


class Lidar:
    """
    Reads the LD06 in a background thread and remembers the newest distance for every
    whole degree. Ask it for the nearest distance inside an angle range with .distance().
    Angles: 0 = robot front, increasing clockwise.
    """

    def __init__(self, ser=None):
        if ser is None:
            import serial                      # pip install pyserial
            ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
        self.ser = ser
        self.bins = [None] * 360               # each entry: (distance_mm, timestamp)
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            for angle, dist_mm, conf in ld06_points(self.ser):
                if not self.running:
                    break
                angle = (angle + ANGLE_OFFSET) % 360.0
                if conf < MIN_CONFIDENCE or in_deadzone(angle):
                    continue
                self.bins[int(angle) % 360] = (dist_mm, time.time())
        except Exception:
            pass                               # port closed while shutting down

    def distance(self, start, end, max_age=0.5):
        """Nearest recent distance (mm) between two angles, or None if nothing was seen."""
        now = time.time()
        best = None
        for a in range(360):
            b = self.bins[a]
            if b and now - b[1] <= max_age and in_sector(a, start, end):
                if best is None or b[0] < best:
                    best = b[0]
        return best

    def close(self):
        self.running = False
        self.ser.close()


# ================================================ 3. GRID PREDICTION (simulated scan) =
def _is_wall(cell):
    return cell == 1 or cell == "1"            # "1" in case the grid came from a CSV


def cast_ray(grid, row, col, map_angle_deg, max_mm=MAX_COMPARE_MM):
    """
    Walk a ray over the grid from the middle of cell (row, col).
    map_angle_deg is clockwise from north: 0 = north, 90 = east.
    Returns the distance in mm to the first wall (or max_mm).
    """
    dr = -math.cos(math.radians(map_angle_deg))
    dc = math.sin(math.radians(map_angle_deg))
    height, width = len(grid), len(grid[0])
    r0, c0 = row + 0.5, col + 0.5
    max_cells = max_mm / CELL_MM

    d = 0.0
    while d < max_cells:
        d += 0.1
        r, c = int(math.floor(r0 + dr * d)), int(math.floor(c0 + dc * d))
        if not (0 <= r < height and 0 <= c < width):
            return d * CELL_MM if BORDER_IS_WALL else max_mm
        if _is_wall(grid[r][c]):
            return d * CELL_MM
    return max_mm


def sector_window(map_sector, heading):
    """
    Lidar angle range (start, end) that looks at map sector `map_sector`
    (0 = north, then clockwise in steps of 15 degrees) while the robot faces `heading`.
    """
    heading_deg = 90 * HEADINGS.index(heading)
    centre = (map_sector * SECTOR_DEG - heading_deg) % 360.0
    return (centre - SECTOR_DEG / 2) % 360.0, (centre + SECTOR_DEG / 2) % 360.0


# ============================================================ 4. TOPOLOGICAL TRACKER =
class TopologicalTracker:
    """
    Compares the real lidar view with the view the grid predicts.
        grid             your map (0 free, 1 wall, "I3" etc. for nodes)
        adjacency_list   from graph.py (only needed for neighbour lookups)
        node_coordinates {"I3": [row, col], ...} from graph.py
        lidar            a Lidar (or anything with .distance(start_deg, end_deg))
    """

    def __init__(self, grid, adjacency_list, node_coordinates, lidar):
        self.grid = grid
        self.adjacency_list = adjacency_list
        self.node_coordinates = node_coordinates
        self.lidar = lidar
        self._cache = {}

    def expected_at_cell(self, row, col):
        """Nearest-wall distance (mm) in each of the 24 map sectors, seen from a cell."""
        key = (row, col)
        if key not in self._cache:
            sig = []
            for s in range(N_SECTORS):
                lo = s * SECTOR_DEG - SECTOR_DEG / 2
                angles = [lo + 0.5 + i for i in range(int(SECTOR_DEG))]   # one ray per degree
                sig.append(min(cast_ray(self.grid, row, col, a) for a in angles))
            self._cache[key] = tuple(sig)
        return self._cache[key]

    def expected(self, node):
        row, col = self.node_coordinates[node]
        return self.expected_at_cell(row, col)

    def observed(self, heading):
        """The same 24 sectors, measured by the real lidar, in map directions."""
        sig = []
        for s in range(N_SECTORS):
            d = self.lidar.distance(*sector_window(s, heading))
            sig.append(None if d is None else min(d, MAX_COMPARE_MM))
        return tuple(sig)

    @staticmethod
    def error(observed, expected):
        """
        Mismatch score over the sectors that have data, or None if too few.
        0 = perfect, 1.0 = just within tolerance, bigger = worse.
        """
        scores = [abs(o - min(e, MAX_COMPARE_MM)) / (TOLERANCE_BASE_MM + TOLERANCE_RELATIVE * min(e, MAX_COMPARE_MM))
                  for o, e in zip(observed, expected) if o is not None]
        if len(scores) < MIN_KNOWN_FRACTION * len(expected):
            return None
        return sum(scores) / len(scores)


# ================================================================= 5. ROUTE CHECKS =
def cells_on_edge(node_coordinates, from_node, to_node):
    """All grid cells (row, col) on the straight line from one node to the next."""
    r0, c0 = node_coordinates[from_node]
    r1, c1 = node_coordinates[to_node]
    dr = (r1 > r0) - (r1 < r0)
    dc = (c1 > c0) - (c1 < c0)
    steps = abs(r1 - r0) + abs(c1 - c0)
    return [(r0 + dr * i, c0 + dc * i) for i in range(steps + 1)]


def check_at_node(tracker, node, heading):
    """
    The route says we are at `node`, facing `heading` ("north"/"east"/"south"/"west").
    Returns (status, score). status is "on_route", "off_route" or "unknown".
    """
    score = tracker.error(tracker.observed(heading), tracker.expected(node))
    if score is None:
        return "unknown", None
    return ("on_route" if score <= MATCH_SCORE else "off_route"), score


def check_on_edge(tracker, from_node, to_node, heading):
    """
    The route says we are driving from `from_node` to `to_node`, facing `heading`.
    The lidar is compared with every cell on that corridor; the best match counts.
    Returns (status, score, best_cell). best_cell only tells real progress where the
    corridor's view changes (near crossings); in a plain straight aisle every cell looks alike.
    """
    obs = tracker.observed(heading)
    best_score, best_cell = None, None
    for row, col in cells_on_edge(tracker.node_coordinates, from_node, to_node):
        score = tracker.error(obs, tracker.expected_at_cell(row, col))
        if score is None:
            return "unknown", None, None
        if best_score is None or score < best_score:
            best_score, best_cell = score, (row, col)
    status = "on_route" if best_score <= MATCH_SCORE else "off_route"
    return status, best_score, best_cell


def closest_nodes(tracker, heading, how_many=3):
    """
    When the check says off_route: which nodes does the lidar view resemble most?
    Returns [(score, node), ...]. Many nodes look alike, so this is a hint, not a position.
    """
    obs = tracker.observed(heading)
    scored = []
    for node in tracker.node_coordinates:
        score = tracker.error(obs, tracker.expected(node))
        if score is not None:
            scored.append((score, node))
    return sorted(scored)[:how_many]


class RouteMonitor:
    """
    Raises an alarm only after several off_route readings in a row, so one noisy scan
    does not stop the robot. "unknown" is ignored; "on_route" resets the count.
    """

    def __init__(self, alarm_after=3):
        self.alarm_after = alarm_after
        self.off_count = 0

    def report(self, status):
        """Feed it each status. Returns True when the robot is really off route."""
        if status == "off_route":
            self.off_count += 1
        elif status == "on_route":
            self.off_count = 0
        return self.off_count >= self.alarm_after


# ============================================================== 6. TESTING HELPER =
def compare_view(tracker, node, heading):
    """
    Place the robot by hand on `node` facing `heading`, then call this. Prints what the
    lidar sees next to what the grid predicts for each 15 degree slice (clockwise from
    north) and the overall score. Use it to tune TOLERANCE_* on the real robot.
    """
    obs = tracker.observed(heading)
    exp = tracker.expected(node)
    print(f"node {node}, facing {heading}")
    print(f"{'map angle':>9} | {'lidar mm':>8} | {'grid mm':>7}")
    for s in range(N_SECTORS):
        o = "-" if obs[s] is None else f"{obs[s]:.0f}"
        print(f"{s * SECTOR_DEG:9.0f} | {o:>8} | {min(exp[s], MAX_COMPARE_MM):7.0f}")
    score = tracker.error(obs, exp)
    print("score:", "not enough lidar data" if score is None else f"{score:.2f} (1.0 or lower = match)")
