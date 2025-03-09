import heapq
import collections

# --- Grid Definition ---
# 1: Boundary, 2: Obstacle, 0: Lane, 3: Starting position.
grid = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 2, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1],
    [1, 2, 0, 0, 1, 1, 1, 0, 0, 0, 1],
    [1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1],
    [1, 3, 0, 0, 1, 1, 1, 0, 0, 0, 1],
    [1, 0, 0, 0, 1, 1, 1, 2, 2, 0, 1],
    [1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1],
    [1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1],
    [1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]

# --- Parameters ---
MIN_LOOP_MOVES = 40       # Loop must be at least 40 moves.
MIN_VISITED_COUNT = 30    # And cover at least 30 distinct allowed cells.
MAX_SPEED = 3             # Maximum speed allowed.

# Directions (0: east, 1: south, 2: west, 3: north)
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]

# --- Helper: Lateral Shift ---
def lateral_shift(d, lateral):
    """
    Returns a lateral shift vector relative to direction d.
    lateral: -1 for left, 0 for no shift, 1 for right.
    """
    if lateral == 0:
        return (0, 0)
    elif lateral == -1:
        return DIRS[(d - 1) % 4]
    else:  # lateral == 1
        return DIRS[(d + 1) % 4]

# --- Helpers for Bounds and Cell Availability ---
def in_bounds(x, y, grid):
    R, C = len(grid), len(grid[0])
    return 0 <= x < R and 0 <= y < C

def cell_free(x, y, grid):
    # Allowed cells are those marked 0; the start cell (3) is converted to 0.
    return grid[x][y] == 0

# --- Precompute Allowed Cells Mapping ---
# Map each coordinate (that is 0 or 3) to a unique index for bitmasking.
allowed_mapping = {}
index = 0
R, C = len(grid), len(grid[0])
for i in range(R):
    for j in range(C):
        if grid[i][j] == 0 or grid[i][j] == 3:
            allowed_mapping[(i, j)] = index
            index += 1

def set_bit(mask, idx):
    return mask | (1 << idx)

def count_bits(mask):
    return bin(mask).count("1")

# --- Find Starting Position ---
def find_start(grid):
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 3:
                return (i, j)
    return None

# --- Heuristic Function for A* ---
def heuristic(x, y, moves, mask, start):
    # Manhattan distance from current cell to start.
    hx = abs(x - start[0]) + abs(y - start[1])
    # Penalty if we haven't reached minimum moves.
    penalty_moves = max(0, MIN_LOOP_MOVES - moves)
    # Penalty if we haven't visited enough distinct cells.
    penalty_visited = max(0, MIN_VISITED_COUNT - count_bits(mask))
    return hx + penalty_moves + penalty_visited

# --- A* Search with Path Reconstruction ---
def a_star_loop(grid):
    start = find_start(grid)
    if start is None:
        return None
    sx, sy = start
    grid[sx][sy] = 0  # Mark start as free.
    init_mask = 0
    if (sx, sy) in allowed_mapping:
        init_mask = set_bit(init_mask, allowed_mapping[(sx, sy)])
    # State: (x, y, speed, direction, moves, mask)
    init_state = (sx, sy, 0, 0, 0, init_mask)
    # Priority queue holds (f, state) where f = moves + heuristic.
    pq = []
    start_h = heuristic(sx, sy, 0, init_mask, start)
    heapq.heappush(pq, (start_h, init_state))
    # Dictionary for path reconstruction: maps state_id to previous state.
    came_from = { (sx, sy, 0, 0, init_mask): None }
    visited_states = set([(sx, sy, 0, 0, init_mask)])
    
    while pq:
        f, (x, y, speed, d, moves, mask) = heapq.heappop(pq)
        # Terminal condition: returned to start with sufficient moves and visited count.
        if (x, y) == (sx, sy) and moves >= MIN_LOOP_MOVES and count_bits(mask) >= MIN_VISITED_COUNT:
            final_state = (x, y, speed, d, moves, mask)
            return reconstruct_path(came_from, final_state), moves, mask
        # Option 1: Move without turning.
        for ds in [-1, 0, 1]:
            new_speed = speed + ds
            if new_speed < 0 or new_speed > MAX_SPEED:
                continue
            for lateral in [-1, 0, 1]:
                fx, fy = DIRS[d]
                disp_forward = (new_speed * fx, new_speed * fy)
                lat_dx, lat_dy = lateral_shift(d, lateral)
                disp_lat = (lat_dx, lat_dy)
                new_x = x + disp_forward[0] + disp_lat[0]
                new_y = y + disp_forward[1] + disp_lat[1]
                if not in_bounds(new_x, new_y, grid):
                    continue
                if not cell_free(new_x, new_y, grid):
                    continue
                new_mask = mask
                if (new_x, new_y) in allowed_mapping:
                    new_mask = set_bit(new_mask, allowed_mapping[(new_x, new_y)])
                new_state = (new_x, new_y, new_speed, d, moves + 1, new_mask)
                state_id = (new_x, new_y, new_speed, d, new_mask)
                if state_id not in visited_states:
                    visited_states.add(state_id)
                    came_from[state_id] = (x, y, speed, d, mask)  # store previous state (without moves)
                    g = moves + 1
                    h = heuristic(new_x, new_y, g, new_mask, start)
                    heapq.heappush(pq, (g + h, new_state))
        # Option 2: If speed is 0 or 1, allow a right turn.
        if speed in [0, 1]:
            new_d = (d + 1) % 4
            for ds in [-1, 0, 1]:
                new_speed = speed + ds
                if new_speed < 0 or new_speed > MAX_SPEED:
                    continue
                for lateral in [-1, 0, 1]:
                    fx, fy = DIRS[new_d]
                    disp_forward = (new_speed * fx, new_speed * fy)
                    lat_dx, lat_dy = lateral_shift(new_d, lateral)
                    disp_lat = (lat_dx, lat_dy)
                    new_x = x + disp_forward[0] + disp_lat[0]
                    new_y = y + disp_forward[1] + disp_lat[1]
                    if not in_bounds(new_x, new_y, grid):
                        continue
                    if not cell_free(new_x, new_y, grid):
                        continue
                    new_mask = mask
                    if (new_x, new_y) in allowed_mapping:
                        new_mask = set_bit(new_mask, allowed_mapping[(new_x, new_y)])
                    new_state = (new_x, new_y, new_speed, new_d, moves + 1, new_mask)
                    state_id = (new_x, new_y, new_speed, new_d, new_mask)
                    if state_id not in visited_states:
                        visited_states.add(state_id)
                        came_from[state_id] = (x, y, speed, d, mask)
                        g = moves + 1
                        h = heuristic(new_x, new_y, g, new_mask, start)
                        heapq.heappush(pq, (g + h, new_state))
    return None

def reconstruct_path(came_from, final_state):
    # Reconstruct the full path including (x,y,speed,direction) for every move.
    path = []
    # The state_id used in came_from is (x, y, speed, direction, mask).
    x, y, speed, d, moves, mask = final_state
    state_id = (x, y, speed, d, mask)
    # Append final state (with moves count).
    path.append((x, y, speed, d))
    while came_from.get(state_id) is not None:
        prev = came_from[state_id]
        # prev is a tuple (prev_x, prev_y, prev_speed, prev_direction, prev_mask)
        path.append((prev[0], prev[1], prev[2], prev[3]))
        state_id = (prev[0], prev[1], prev[2], prev[3], prev[4])
    path.reverse()
    return path

def main():
    # Work on a copy of grid.
    result = a_star_loop([row[:] for row in grid])
    if result:
        path, moves, mask = result
        print("Minimum moves required to complete one clockwise round:", moves)
        print("Distinct visited count:", count_bits(mask))
        print("\nDetailed Path (x, y, speed, direction):")
        for idx, state in enumerate(path):
            print(f"Move {idx}: {state}")
    else:
        print("No valid loop found.")

if __name__ == '__main__':
    main()
