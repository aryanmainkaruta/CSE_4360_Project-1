import heapq
import math

MAX_OBSTACLES = 25
num_obstacles = 13;
obstacle = [(0.61, 2.743), (0.915, 2.743), (1.219, 2.743), (1.829, 1.219),
            (1.829, 1.524), (1.829, 1.829), (1.829, 2.134), (2.743, 0.305),
            (2.743, 0.61), (2.743, 0.915), (2.743, 2.743), (3.048, 2.743),
            (3.353, 2.743)]
start = (0.305, 1.219)  # Start location
goal = (3.658, 1.829)   # 

def heuristic(a, b):
    
    """ 
        Heuristic function: Euclidean distance between two points a and b. 
        Uses the Euclidean distance to calculate the shortest distance between two points.
    """
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    
def is_in_obstacle(x, y):
    """ 
        Check if the point (x, y) lies within any obstacle. 
        Checks if a given point is within an obstacle area by ensuring the distance from obstacles is greater than 0.61 meters (2 tiles wide).
    """
    for obs in obstacle:
        if heuristic((x, y), obs) < 0.61:  # Ensure at least 0.61m distance from obstacles
            return True
    return False

    
def neighbors(node):
    """ 
        Returns the neighboring nodes (up, down, left, right). 
    
    """
    
def astar(start, goal):
    """ 
        A* algorithm implementation.
        Checks if a given point is within an obstacle area by ensuring the distance from obstacles is greater than 0.61 meters (2 tiles wide).
    
    """