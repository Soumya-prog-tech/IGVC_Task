#!/home/soumya/miniconda3/envs/ml_pclub_roadmap/bin/python
from collections import deque

def read_matrix():
    # Test case matrix
    matrix = [
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
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    ]
    return matrix

def find_start_position(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 3:
                return (i, j)
    return None

def main():
    matrix = read_matrix()
    start_pos = find_start_position(matrix)
    if not start_pos:
        print("No starting position found.")
        return
    
    # The track is assumed to be generated here. For the sake of example, we'll pretend it's a list of positions.
    # This is a placeholder and would need to be implemented to trace the actual track.
    track = [start_pos]  # Example; actual track generation required.
    track_length = len(track)
    if track_length == 0:
        print("No track found.")
        return
    
    # Directions and lanes would need to be determined based on track's actual path.
    # For this example, assume lanes are adjacent columns.
    # This is a simplification and may not work for all cases.
    lanes = 3
    start_lane = 1  # Assuming starting in middle lane
    
    # BFS initialization
    visited = set()
    queue = deque()
    initial_speed = 0
    # State: (track_index, lane, speed)
    initial_state = (0, start_lane, initial_speed)
    queue.append((initial_state, 0))
    visited.add(initial_state)
    
    found = False
    while queue:
        (current_idx, current_lane, current_speed), moves = queue.popleft()
        
        # Check if we've completed a full round
        if current_idx % track_length == 0 and moves > 0:
            # Check if back to start position and lane
            if current_idx % track_length == 0 and current_lane == start_lane:
                print(f"Minimum moves required: {moves}")
                found = True
                break
        
        # Generate next possible states
        for delta_speed in [-1, 0, 1]:
            new_speed = current_speed + delta_speed
            if new_speed < 0:
                continue  # Speed cannot be negative
            
            for delta_lane in [-1, 0, 1]:
                new_lane = current_lane + delta_lane
                if new_lane < 0 or new_lane >= lanes:
                    continue  # Lane out of bounds
                
                # Check if direction change is allowed (if applicable)
                # This part is skipped as the problem's direction change rules are unclear
                
                # Calculate new track index after moving 'new_speed' steps
                new_idx = (current_idx + new_speed) % track_length
                
                # Check if all steps in the move are valid
                valid = True
                for step in range(1, new_speed + 1):
                    idx = (current_idx + step) % track_length
                    pos = track[idx]
                    # Determine lane's actual position. Simplified to pos's row and column adjusted by lane
                    # This is a placeholder and needs proper calculation based on track direction
                    lane_pos = (pos[0], pos[1] + (new_lane - 1))  # Simplified example
                    if lane_pos[0] < 0 or lane_pos[0] >= len(matrix) or lane_pos[1] < 0 or lane_pos[1] >= len(matrix[0]):
                        valid = False
                        break
                    cell_value = matrix[lane_pos[0]][lane_pos[1]]
                    if cell_value == 2 or cell_value == 0:
                        valid = False
                        break
                if not valid:
                    continue
                
                new_state = (new_idx, new_lane, new_speed)
                if new_state not in visited:
                    visited.add(new_state)
                    queue.append((new_state, moves + 1))
    
    if not found:
        print("No valid path found.")

if __name__ == "__main__":
    main()