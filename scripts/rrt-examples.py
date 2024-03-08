#!/usr/bin/env python3

# Moved the example code from main in rrt.py

import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ca2_ttk4192.srv import isThroughObstacle, isThroughObstacleRequest, isInObstacle, isInObstacleRequest, positionControl, positionControlRequest


if __name__ == '__main__':
 # -----------------
    # Init the RRT node
    rospy.init_node('RRT')

    # -----------------------------------------------
    # The start and end positions of the "short maze"
    startpos = (0.0, 0.0)
    endpos = (4.5, 5.0)

    # -------------------------------------------------------------------------
    # The start and end positions of the bonus task with the "complicated maze"
    startpos = (0.0, 0.0)
    endpos = (4.5, 9.0)
    
    # ---------------------------------------------------------------
    # Example of how you can check if a point is inside an obstacle: 

    obstacle_radius = 0.3
    vex_pos = [0.0, 0.0]
    response = isInObstacle(vex_pos, obstacle_radius)
    print("point_in_obstacle_service response: ")
    print(response)
    
    # -----------------------------------------------------------------------
    # Example of how you can check if the straightline path between two points 
    # goes through an obstacle:

    obstacle_radius = 0.3 # margin
    first_point = [0.0, 0.0]
    second_point = [1.0, 1.0]
    response = isThruObstacle(first_point, second_point, obstacle_radius)
    print("path_through_obstacle_service response: ")
    print(response)

    # -----------------------------------------------------------------------
    # Example of how you can visualize a graph using a MarkerArray publisher:

    list_of_positions = [[0.0, 0.0], [0.0, 2.0], [1.0, 2.0], [1.0, 1.0]]

    tree_publisher = rospy.Publisher('tree_marker', MarkerArray, queue_size=10)
    tree_marker = MarkerArray()
    marker_identity = 0

    # Create a blue-green square representing the start
    start_rgb_color = [0/256, 158/256, 115/256]
    start_marker_size = 0.2
    start_marker = get_marker(Marker.CUBE, list_of_positions[0], start_marker_size, start_rgb_color, marker_identity)
    marker_identity += 1

    # Create a vermillion square representing the goal
    end_rgb_color = [213/256, 94/256, 0/256]
    end_marker_size = 0.2
    end_marker = get_marker(Marker.CUBE, list_of_positions[-1], end_marker_size, end_rgb_color, marker_identity)
    marker_identity +=1

    tree_marker.markers.append(start_marker)
    tree_marker.markers.append(end_marker)

    
    # Create reddish purple edges 
    edge_color = [204/256, 121/256, 167/256]
    
    for index in range(len(list_of_positions)-1):
        first_point = list_of_positions[index]
        second_point = list_of_positions[index+1]
        edge_marker = get_edge_as_marker(first_point, second_point, edge_color, marker_identity)
        marker_identity += 1
        tree_marker.markers.append(edge_marker)


    rospy.Rate(0.5).sleep() # needs to a bit for the publisher to start, a bit weird. 
    tree_publisher.publish(tree_marker)



    # ---------------------------------------------------------------------------
    # Example of how you can make the turtlebot go through a sequence of positions
    # using the position controller.

    # Note that it will just move in a straight line between the current position and the
    # desired position, so it will crash into possible obstacles. 
    # It is also not very well tuned (and quite slow).


    list_of_position = [[0.0, 2.0], [1.0, 2.0], [1.0, 1.0]]
    position_control = rospy.ServiceProxy('/position_control', positionControl)

    for position in list_of_position:
        rospy.wait_for_service('/position_control')
        request = Point(position[0], position[1], 0.0)
        response = position_control(request)
        print(response)