import time
import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait

# Initialize the EV3 Brick and motors
ev3 = EV3Brick()

# Motors for movement
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Sensors
color_sensor = ColorSensor(Port.S1)
touch_sensor = TouchSensor(Port.S2)

MAX_OBSTACLES = 25
num_obstacles = 13

step_size = 0.305  # Robot movement step size (half of a tile)

# 2D environment grid (0 = empty space, 1 = obstacle, 2 = fire)
grid = [
    [0, 0, 0, 1, 0, 0, 0],  # Row 0
    [1, 1, 0, 0, 0, 1, 0],  # Row 1
    [0, 0, 0, 1, 0, 2, 0],  # Row 2 (2 represents fire)
    [0, 1, 0, 0, 0, 0, 1],  # Row 3
    [0, 0, 0, 1, 1, 0, 0]   # Row 4
]

# Initial setup: starting at top-left corner
position = [0, 0]  # Robot's starting position
goal_position = [2, 5]  # Position of the fire (goal)
goal_reached = False  # Boolean to check if goal is reached
on_m_line = True  # Bool to check whether the robot is following the M-line 

# Define obstacles (hardcoded for the environment)
obstacle_positions = [(0, 3), (1, 0), (1, 1), (3, 1), (3, 6), (4, 3), (4, 4)]

def calculate_distance(point1, point2):
    """Calculate the straight-line distance between two points in the 2D plane."""
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def check_if_obstacle(x, y):
    """Verify if a given point (x, y) is too close to any defined obstacles."""
    for obs in obstacle_positions:
        if calculate_distance((x, y), obs) < 0.61:  # 0.61 meters represents the size of an obstacle
            return True
    return False

# Function to move forward towards the goal along the M-line
def wander():
    global position, on_m_line
    if on_m_line:
        print("Following M-line towards goal...")
        # Move towards goal by adjusting position along the M-line
        if position[0] < goal_position[0]:  # If the current row is less than the goal's row
            left_motor.run_time(200, 1000)  # Move down
            right_motor.run_time(200, 1000)
            position[0] += 1  # Update position
        elif position[0] > goal_position[0]:  # If the current row is greater than the goal's row
            left_motor.run_time(-200, 1000)  # Move up
            right_motor.run_time(-200, 1000)
            position[0] -= 1  # Update position

        elif position[1] < goal_position[1]:  # If the current column is less than the goal's column
            left_motor.run_time(200, 1000)  # Move right
            right_motor.run_time(-200, 1000)
            position[1] += 1  # Update position

        elif position[1] > goal_position[1]:  # If the current column is greater than the goal's column
            left_motor.run_time(-200, 1000)  # Move left
            right_motor.run_time(200, 1000)
            position[1] -= 1  # Update position
        time.sleep(1)
        print(f"New position on M-line: {position}")

# Function to check if there is an obstacle in front
def obstacle_in_front():
    next_pos = list(position)  # Copy current position
    # Move the robot one step in the direction of the goal
    if next_pos[0] < goal_position[0]:
        next_pos[0] += 1
    elif next_pos[0] > goal_position[0]:
        next_pos[0] -= 1
    elif next_pos[1] < goal_position[1]:
        next_pos[1] += 1
    elif next_pos[1] > goal_position[1]:
        next_pos[1] -= 1

    # Check if there's an obstacle at the next position (for the current simulation )
    if grid[next_pos[0]][next_pos[1]] == 1:
        print("Obstacle detected")
        return True  # Obstacle detected
    return False  # No obstacle

# Function to detect fire based on proximity to the goal position
def detect_fire():
    # Calculate the Euclidean distance to the goal
    distance_to_goal = ((goal_position[0] - position[0]) ** 2 + (goal_position[1] - position[1]) ** 2) ** 0.5
    if distance_to_goal < 1:  # If close enough to goal (fire)
        print("Fire detected!")
        return True
    return False

# Function to simulate extinguishing fire
def extinguish():
    print("Extinguishing the fire...")

# Function to simulate wall-following behavior
def wall_follow():
    print("Following the wall...")

# Main function to manage robot behaviors
def robot_controller():
    global goal_reached, on_m_line

    # Main loop to run the robot's behaviors
    while not goal_reached:
        if detect_fire():  # Check if fire is detected
            extinguish()  # Extinguish fire if detected
            goal_reached = True  # Mark goal as reached
            print("Mission complete.")
            break  # Exit the loop when the mission is complete

        if on_m_line:  # If robot is on the M-line, it tries to wander towards the goal
            wander()  # Move towards the goal
            if obstacle_in_front():  # Check if an obstacle is encountered
                on_m_line = False  # Switch to wall-following mode if obstacle is found
                print("Obstacle encountered, switching to wall-following.")
        else:  # If not on M-line, attempt to follow the wall
            print("Wall-following behavior activated.")
            wall_follow()  # Call the wall-following behavior

# Main loop to run the behavior controller
if __name__ == "__main__":
    robot_controller()  # Start the robot controller which handles behaviors
