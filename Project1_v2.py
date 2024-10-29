import math
#from astar_methods import *          # Import additional A* methods, if necessary
#from CalibrationTest import *         # Import robot movement functions
import matplotlib.pyplot as plt

# Define obstacle, start, and goal parameters
obstacle_positions = [
    (0.61, 2.743), (0.915, 2.743), (1.219, 2.743), (1.829, 1.219),
    (1.829, 1.524), (1.829, 1.829), (1.829, 2.134), (2.743, 0.305),
    (2.743, 0.61), (2.743, 0.915), (2.743, 2.743), (3.048, 2.743),
    (3.353, 2.743)
]
start_position = (0.305, 1.219)
goal_position = (3.658, 1.829)
obstacle_radius = 0.61
tile_size = 0.305
goal_tolerance = 0.01
worldX=4.88
worldY=3.05
robotRadius=(8/12)*0.305 # Sets extra buffer around objects to avoid the robot scraping the side of objects
subdivison=2 # How many times to split the tiles (technically increases processing time and memory)

# A* Pathfinding and Visualization Functions
def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def check_if_obstacle_circle(x, y):
    return any(calculate_distance((x, y), obs) < tile_size for obs in obstacle_positions)

# Assumes Square Obstacle
def check_if_obstacle(x, y): 
    return any(
        abs(x - obs[0]) < (tile_size / 2 + robotRadius) and abs(y - obs[1]) < (tile_size / 2+robotRadius)
        for obs in obstacle_positions
    )


def generate_neighbors(node):
    x, y = node
    step = tile_size / subdivison
    possible_moves = [
        (x + step, y), (x - step, y),
        (x, y + step), (x, y - step)
    ]
    return [move for move in possible_moves if not check_if_obstacle(move[0], move[1])]

def is_goal_reached(current, goal, tolerance):
    return calculate_distance(current, goal) < tolerance

def a_star_search(start, goal):
    if check_if_obstacle(start[0], start[1]) or check_if_obstacle(goal[0], goal[1]):
        return None
    to_explore = [(start, calculate_distance(start, goal))]
    path_tracking, cost_to_reach = {}, {start: 0}
    while to_explore:
        to_explore.sort(key=lambda x: x[1])
        current_node, _ = to_explore.pop(0)
        if is_goal_reached(current_node, goal, goal_tolerance):
            path = []
            while current_node in path_tracking:
                path.append(current_node)
                current_node = path_tracking[current_node]
            path.reverse()
            return path
        for neighbor in generate_neighbors(current_node):
            new_cost = cost_to_reach[current_node] + calculate_distance(current_node, neighbor)
            if neighbor not in cost_to_reach or new_cost < cost_to_reach[neighbor]:
                path_tracking[neighbor] = current_node
                cost_to_reach[neighbor] = new_cost
                estimated_total = new_cost + calculate_distance(neighbor, goal)
                to_explore.append((neighbor, estimated_total))
    return None

def plot_environment(path, obstacles, start, goal, subdivisions=subdivison):
    path.insert(0,start)
    fig, ax = plt.subplots()

    for obs in obstacles:
        # ax.add_artist(plt.Circle(obs, obstacle_radius, color='black', fill=False, linewidth=3))
        # Add Obstacles
        # Obstacles are constructed from 1’x1’ cardboard square
        ax.add_artist(plt.Rectangle((obs[0]-tile_size/2,obs[1]-tile_size/2), tile_size,tile_size, color='black', fill=False, linewidth=3))
    if path:
        x_path, y_path = zip(*path)
        ax.plot(x_path, y_path, color='red', linewidth=2, label='Path')
    ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
    ax.plot(goal[0], goal[1], 'bo', markersize=10, label='Goal')
    # The goal ismarked by a black circle with a radius of 1’ (0.305m)
    ax.add_artist(plt.Circle(goal, tile_size, color='blue', fill=False, linewidth=3))
    # Plot World Bounding Box
    # The workspace is a rectangular area measuring 16’ x 10’ (4.88m x 3.05m).
    ax.add_artist(plt.Rectangle((0,0),worldX,worldY, color='purple', fill=False, linewidth=3))
    
    ax.set_aspect('equal', adjustable='box')
    
    x_min = min(min(x_path), start[0], goal[0]) - 0.5
    x_max = max(max(x_path), start[0], goal[0]) + 0.5
    y_min = min(min(y_path), start[1], goal[1]) - 0.5
    y_max = max(max(y_path), start[1], goal[1]) + 0.5
    
    # plt.xlim(x_min, x_max)
    # plt.ylim(y_min, y_max)
    plt.xlim(-1, 4.88+1)
    plt.ylim(-1, 3.05+1)
    
    ax.grid(True)
    # for x in range(int(x_min / tile_size) - 1, int(x_max / tile_size) + 1):
    #     for y in range(int(y_min / tile_size) - 1, int(y_max / tile_size) + 1):
    #         if not check_if_obstacle(x * tile_size, y * tile_size):
    #             plt.plot(x * tile_size, y * tile_size, 'x', color='cyan', markersize=5)
    for x in range(0, int(worldX*1000), int(tile_size*1000/subdivisions)):
        for y in range(0, int(worldY*1000), int(tile_size*1000/subdivisions)):
            if not check_if_obstacle(x/1000 , y/1000):
                plt.plot(x/1000, y/1000, 'x', color='cyan', markersize=5)

    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.show()

# Run A* and plot results
path_result = a_star_search(start_position, goal_position)
plot_environment(path_result, obstacle_positions, start_position, goal_position,subdivisions=subdivison)

#Execute robot movement commands along the path
if path_result:
    for i in range(1, len(path_result)):
        x1, y1 = path_result[i - 1]
        x2, y2 = path_result[i]
        travel_angle = math.atan2(y2 - y1, x2 - x1)
        travel_distance = calculate_distance((x1, y1), (x2, y2))
        TurnForAngle(travel_angle)          # Turn robot towards next segment
        MoveStraightForDistance(travel_distance)  # Move robot to the next position
else:
    print("No valid path could be found.")
