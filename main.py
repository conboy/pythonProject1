#!/usr/bin/env python2.7

from qset_lib import Rover
import math

grid_point_size = 1
grid_extra_size = 5
heading_latency = 1
angular_speed = 0.8
linear_speed = 0.5
rover = Rover()


# Creates a grid with corners point1 and point2
def create_grid(point1, point2):
    graph_points = []

    # Nested for loop for adding all the points to the list from the min to max of corner points
    # Also adds a bit extra to the grid size to be sure a path can be made
    for x in range(int(min(point1[0], point2[0]) - grid_extra_size), int(max(point1[0], point2[0]) + grid_extra_size)):
        for y in range(int(min(point1[1], point2[1]) - grid_extra_size), int(max(point1[1], point2[1]) + grid_extra_size)):
            graph_points.append((x, y))

    return graph_points


# Gets the grid point of a gps coordinate, using grid point size
def gps_to_point(point):
    return math.floor(point[0] / grid_point_size), math.floor(point[1] / grid_point_size)


# Returns a list of grid points that the rover is currently seeing
def lidar_to_grid():
    rover_laser_distances = rover.laser_distances
    rover_obstacle_distances = []

    obstacle_coordinates = []
    angle_between_lasers = 90.0 / 14.0
    initial_angle = rover.heading

    for i in range(len(rover_laser_distances)):
        if rover_laser_distances[i] != float("inf"):
            laser_distance = rover_laser_distances[i]
            laser_number = i
            laser_angle = initial_angle + angle_between_lasers * laser_number
            obstacle_x = laser_distance * math.sin(laser_angle)
            obstacle_y = laser_distance * math.cos(laser_angle)
            obstacle_coordinates.append(gps_to_point((obstacle_x, obstacle_y)))

    return obstacle_coordinates


# Drives the rover in a straight line to a gps point
def go_to_point(target_point):
    current_point = (rover.x, rover.y)
    target_heading = math.atan2(target_point[1] - current_point[1], target_point[0] - current_point[0]) * (
                180 / math.pi)
    if target_heading < 0:
        target_heading += 360
    print("target heading", target_heading)
    # While the rover heading is not within the set latency, turn towards the right direction
    while True:
        rover_heading = rover.heading
        if rover_heading < 0:
            rover_heading += 360

        if rover_heading > target_heading - heading_latency and rover_heading < target_heading + heading_latency:
            break
        else:
            rover.send_command(0, angular_speed)

    rover.send_command(0, 0)

    while True:
        #print("Rover gps: " + str(rover.x) + "," + str(rover.y) + " - Target gps: " + str(target_point))
        # If the rover is now past or at the target gps
        if (current_point[0] < target_point[0] <= rover.x or current_point[0] > target_point[0] >= rover.x or current_point[0] - target_point[0]==0) and \
                (current_point[1] < target_point[1] <= rover.y or current_point[1] > target_point[1] >= rover.y or current_point[1] - target_point[1]==0):
            print('rover should stop')
            break
        else:
            rover.send_command(linear_speed, 0)

    rover.send_command(0, 0)


# Returns a path from start point to target point through the list of grid points
def dijkstras(points, start_point, target_point):
    shortest_distance = {}
    predecessor = {}
    unseenPoints = points
    infinity = 9999999
    path = []

    for point in unseenPoints:
        shortest_distance[point] = infinity
    shortest_distance[start_point] = 0

    while unseenPoints:
        minPoint = None
        for point in unseenPoints:
            if minPoint is None:
                minPoint = point
            elif shortest_distance[point] < shortest_distance[minPoint]:
                minPoint = point

        childPoints = {(minPoint[0] + 1, minPoint[1]): 1,
                       (minPoint[0], minPoint[1] + 1): 1,
                       (minPoint[0] - 1, minPoint[1]): 1,
                       (minPoint[0], minPoint[1] - 1): 1}

        for childPoint, weight in childPoints.items():
            if childPoint in points:
                if weight + shortest_distance[minPoint] < shortest_distance[childPoint]:
                    shortest_distance[childPoint] = weight + shortest_distance[minPoint]
                    predecessor[childPoint] = minPoint
        unseenPoints.remove(minPoint)

    currentPoint = target_point
    while currentPoint != start_point:
        try:
            path.insert(0, currentPoint)
            currentPoint = predecessor[currentPoint]
        except KeyError:
            print ("No path")
            break

    return path


# Main function for going to any target gps while avoiding obstacles
def navigate_to_point(target_gps):
    origin_gps = (rover.x, rover.y)

    origin_grid = gps_to_point(origin_gps)
    target_grid = gps_to_point(target_gps)

    grid_points = create_grid(origin_grid, target_grid)

    # Get the initial obstacle data from lidar and remove the points from the grid points if they are in it
    obstacles = lidar_to_grid()
    for point in obstacles:
        if point in grid_points:
            grid_points.remove(point)

    # Run dijkstras with the initial information and get a list containing the path points to follow
    path = dijkstras(grid_points, origin_grid, target_grid)

    # Infinite loop that will just run until broken
    for point in path:
        print(str(point))
    while True:

        # If the path list isn't empty run, if it is empty break the loop because rover is probably at target location or it broke
        if path:
            next_point = path[0]
            print(str(next_point))
            go_to_point((next_point[0] * grid_point_size, next_point[1] * grid_point_size))
            path.pop(0)

            # If the path list is now empty it is assumed that the rover is now at target location, so stop the loop
            if len(path) == 0:
                break

            # Get lidar information from new location
            lidar_grid = lidar_to_grid()
            for lidar_point in lidar_grid:
                # If the obstacle is still in the grid points list remove it
                if lidar_point in grid_points:
                    grid_points.remove(lidar_point)
                    # If the point was also in the path the rover needs to follow, recalculate the path with new information
                    if lidar_point in path:
                        path = dijkstras(grid_points, gps_to_point((rover.x, rover.y)), target_grid)
        else:
            break

    return



navigate_to_point((5,3))
