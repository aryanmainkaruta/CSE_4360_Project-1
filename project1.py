from astar_methods import *
from CalibrationTest import *

obstacle_positions = [(0.61, 2.743), (0.915, 2.743), (1.219, 2.743), (1.829, 1.219),
                      (1.829, 1.524), (1.829, 1.829), (1.829, 2.134), (2.743, 0.305),
                      (2.743, 0.61), (2.743, 0.915), (2.743, 2.743), (3.048, 2.743),
                      (3.353, 2.743)]

start_position = (0.305, 1.219)  # Starting point of the robot
goal_position = (3.658, 1.829)   # Target point the robot aims to reach
goal_tolerance = 0.01  # Tolerance for floating-point comparison

movements=a_star_search(start=start_position,goal=goal_position)

for movement in movements:
    x1=movement[0]
    y1=movement[1]
    x2=movement[2]
    y2=movement[3]
    travel_angle=umath.atan2(x2-x1,y2-y1)
    travel_distance=umath.sqrt(umath.pow((x2-x1),2)+umath.pow((y2-y1),2))

    TurnForAngle(travel_angle)
    MoveStraightForDistance(travel_distance)

