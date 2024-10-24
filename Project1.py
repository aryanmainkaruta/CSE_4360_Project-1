import math

# Maximum number of obstacles and their coordinates
MAX_OBSTACLES = 25
num_obstacles = 13
obstacle_positions = [
    (0.61, 2.743), (0.915, 2.743), (1.219, 2.743), (1.829, 1.219),
    (1.829, 1.524), (1.829, 1.829), (1.829, 2.134), (2.743, 0.305),
    (2.743, 0.61), (2.743, 0.915), (2.743, 2.743), (3.048, 2.743),
    (3.353, 2.743)
]

start_position = (0.305, 1.219)  # Starting point of the robot
goal_position = (3.658, 1.829)    # Target point the robot aims to reach
goal_tolerance = 0.01              # Tolerance for floating-point comparison

def calculate_distance(point1, point2):
    """Calculate the straight-line distance between two points in the 2D plane."""
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def check_if_obstacle(x, y):
    """Verify if a given point (x, y) is too close to any defined obstacles."""
    for obs in obstacle_positions:
        if calculate_distance((x, y), obs) < 0.61:
            return True
    return False

def generate_neighbors(node):
    """Produce a list of potential neighboring points from the current location (node)."""
    x, y = node
    step = 0.305  # Movement step size (half of a tile)
    possible_moves = [
        (x + step, y), (x - step, y),
        (x, y + step), (x, y - step)
    ]
    
    # Filter out any neighboring points that land inside obstacles
    valid_moves = []
    for move in possible_moves:
        if not check_if_obstacle(move[0], move[1]):
            valid_moves.append(move)
    
    return valid_moves

def is_goal_reached(current, goal, tolerance):
    """Check if the current point is within a certain tolerance of the goal."""
    return calculate_distance(current, goal) < tolerance

def a_star_search(start, goal):
    """Implementation of the A* algorithm using a list instead of a priority queue."""
    # Check if start or goal is inside an obstacle
    if check_if_obstacle(start[0], start[1]):
        print("Start point is inside an obstacle!")
        return None
    if check_if_obstacle(goal[0], goal[1]):
        print("Goal point is inside an obstacle!")
        return None

    # Initialize the list for exploration with the starting point
    to_explore = [(start, calculate_distance(start, goal))]  # (node, f_score)

    # Structures to track the best known paths and costs
    path_tracking = {}
    cost_to_reach = {start: 0}  # g_score: cost from start to current

    while to_explore:
        # Sort to get the node with the lowest total cost estimate
        to_explore.sort(key=lambda x: x[1])
        current_node, current_priority = to_explore.pop(0)

        # Debugging: Show the current node being explored
        print(f"Exploring node: {current_node} with priority {current_priority}")

        # If we've reached the goal, reconstruct the path by backtracking
        if is_goal_reached(current_node, goal, goal_tolerance):
            complete_path = []
            while current_node in path_tracking:
                complete_path.append(current_node)
                current_node = path_tracking[current_node]
            complete_path.reverse()  # Reverse the path to start from the beginning
            return complete_path

        # Examine each neighboring node
        for neighbor in generate_neighbors(current_node):
            new_cost = cost_to_reach[current_node] + calculate_distance(current_node, neighbor)

            # Only consider this neighbor if we've found a cheaper path to it
            if neighbor not in cost_to_reach or new_cost < cost_to_reach[neighbor]:
                path_tracking[neighbor] = current_node  # Track the best path to this neighbor
                cost_to_reach[neighbor] = new_cost
                estimated_total = new_cost + calculate_distance(neighbor, goal)  # g + heuristic

                # Add the neighbor to the exploration list
                to_explore.append((neighbor, estimated_total))

    return None  # Return None if no valid path is found

# Execute the A* algorithm to find a valid path from start to goal
path_result = a_star_search(start_position, goal_position)

if path_result:
    print("Path successfully found:")
    for position in path_result:
        print(f"({position[0]:.4f}, {position[1]:.4f})")  # Format to 4 decimal places
else:
    print("No valid path could be found.")
