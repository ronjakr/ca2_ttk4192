#!/usr/bin/env python3

import rospy
from collision_detection import distance
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ca2_ttk4192.srv import isThroughObstacle, isThroughObstacleRequest, isInObstacle, isInObstacleRequest, positionControl, positionControlRequest


def random_configuration():
    """
    Samples a random point in space within the maze
    """

    # Define the range of x and y coordinates - undersøk manuelt i gazebo
    x_min, x_max = 0, 10  # Example range for x coordinates
    y_min, y_max = 0, 10  # Example range for y coordinates

    # Number of samples
    num_samples = 1

    # Generate random x and y coordinates uniformly
    x = np.random.uniform(x_min, x_max, num_samples)
    y = np.random.uniform(y_min, y_max, num_samples)
    return (x,y)


def nearest_vertex(q_random, node_dict):
    """
    Finding the node that is closest to q_random
    """
    nearest_vertex = min(node_dict, key=lambda node: distance(q_random, node))
    # vurder numpy array
    # kan finne nærmeste node som kan kobles til kollisjonsfritt - dersom nærmeste ikke ka ndet, sjekk nest nærmeste
    return nearest_vertex


def scale_configuration(q_nearest, q_random, delta_q):
    """
    Scaling distance between q_nearest and q_random to be
    maximum equal to the maximum distance delta_q
    """

    dist = distance(q_nearest, q_random)

    # If the distance is greater than max distance, create a new point
    if dist > delta_q:
    # Calculate the direction vector
        direction = (q_random - q_nearest) / dist
        
        # Scale the direction vector by max distance
        scaled_direction = direction * delta_q
        
        # Calculate the scaled point
        q_new = q_random + scaled_direction

    return q_new


def rrt_planner(q_init, q_end, delta_q) -> MarkerArray:  # K: number of vertices in RRT
    obstacle_radius = 0.3   # safe margin around obstacles
    distance_threshold = 0.3    # threshold for sufficient close to goal
    run = True

    marker_identity = 0 # initial marker identity
    node_dict[q_init] = None
    marker_identity = visualize_node(q_init, marker_identity)

    while run:
        q_random = random_configuration()   # a random point in space
        q_nearest = nearest_vertex(q_random, node_dict)    # point in "tree" that is closest to the random point
        q_new = scale_configuration(q_nearest, q_random, delta_q)    # modifying configuration in case q_random is too far from q_nearest
        
        if not (isInObstacle(q_new, obstacle_radius) and isThruObstacle(q_new, q_nearest, obstacle_radius)):
            node_dict[q_new] = q_nearest
            marker_identity = visualize_node(q_new, marker_identity)
            marker_identity = visualize_connection(q_new, q_nearest, marker_identity)
            if distance(q_new, q_end) <= distance_threshold:
                run = False

    return node_dict


def visualize_connection(node, parent, marker_identity):
    # Create reddish purple edges 
    edge_color = [204/256, 121/256, 167/256]

    edge_marker = get_edge_as_marker(node, parent, edge_color, marker_identity)
    marker_identity += 1
    tree_marker.markers.append(edge_marker)

    rospy.Rate(0.5).sleep() # needs to a bit for the publisher to start, a bit weird. 
    tree_publisher.publish(tree_marker)
    
    return marker_identity



def visualize_node(node_pos, marker_identity, bool_start=False, bool_goal=False):

    if bool_start == True:
        # Create a blue-green square representing the start
        start_rgb_color = [0/256, 158/256, 115/256]
        start_marker_size = 0.2
        start_marker = get_marker(Marker.CUBE, node_pos, start_marker_size, start_rgb_color, marker_identity)
        tree_marker.markers.append(start_marker)
    
    if bool_goal == True:
        # Create a vermillion square representing the goal
        end_rgb_color = [213/256, 94/256, 0/256]
        end_marker_size = 0.2
        end_marker = get_marker(Marker.CUBE, node_pos, end_marker_size, end_rgb_color, marker_identity)
        marker_identity +=1
        tree_marker.markers.append(end_marker)

    if (bool_start == False and bool_goal == False):
        node_rgb_color = [0/256, 0/256, 256/256]
        node_marker_size = 0.2
        node_marker = get_marker(Marker.CYLINDER, node_pos, node_marker_size, node_rgb_color, marker_identity)
        tree_marker.markers.append(node_marker)

    rospy.Rate(0.5).sleep() # needs to a bit for the publisher to start, a bit weird. 
    tree_publisher.publish(tree_marker)

    marker_identity += 1
    return marker_identity


def positions_from_node_dict(node_dictionary, start_node, goal_node):
    """
    Returns the array of positions for the robot to follow
    """

    positions = []
    positions.append(goal_node)
    node = goal_node
    while not (node == start_node):
        parent = node_dictionary[node]
        positions.insert(0, parent)
        node = parent

    positions.append(start_node)
    return positions



point_in_obstacle_service = rospy.ServiceProxy('point_in_obstacle', isInObstacle)

def isInObstacle(vex, radius):

    vex_pos = Point(vex[0], vex[1], 0.0)

    request = isInObstacleRequest(vex_pos, radius)
    response = point_in_obstacle_service(request)   # calling on service

    return response



path_through_obstacle_service = rospy.ServiceProxy('path_through_obstacle', isThroughObstacle)

def isThruObstacle(p0, p1, radius):

    p0_pos = Point(p0[0], p0[1], 0.0)
    p1_pos = Point(p1[0], p1[1], 0.0)

    request = isThroughObstacleRequest(p0_pos, p1_pos, radius)
    response = path_through_obstacle_service(request)   # calling on service

    return response

def get_marker(type, pos, size, color, identity):

    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = type
    marker.id = identity
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.action = Marker.ADD

    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = 0.001
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    return marker


def get_edge_as_marker(first_point, second_point, color, identity, thickness=0.025):

    edge_marker = get_marker(Marker.LINE_STRIP, [0,0], thickness, color, identity)
    
    p0_point = Point(first_point[0], first_point[1], 0.0)
    p1_point = Point(second_point[0], second_point[1], 0.0)
    edge_marker.points.append(p0_point)
    edge_marker.points.append(p1_point)
    
    return edge_marker


if __name__ == '__main__':
    # Init the RRT node
    rospy.init_node('RRT')

    startpos = (0.0, 0.0)
    endpos = (4.5, 5.0)
    max_incremental_distance = 3  # meters i guess

    node_dict = {}  # key: node, value: parent-node

    tree_publisher = rospy.Publisher('tree_marker', MarkerArray, queue_size=10)
    tree_marker = MarkerArray()

    G = rrt_planner(q_init=startpos, q_end=endpos, delta_q=max_incremental_distance)    # dictionary


    # ---------------------------------------------------------------------------
    # Make the turtlebot go through a sequence of positions using the position controller.

    # Note that it will just move in a straight line between the current position and the
    # desired position, so it will crash into possible obstacles. 
    # It is also not very well tuned (and quite slow).


    list_of_position = positions_from_node_dict(G, start_node=startpos, goal_node=endpos)
    position_control = rospy.ServiceProxy('/position_control', positionControl)

    for position in list_of_position:
        rospy.wait_for_service('/position_control')
        request = Point(position[0], position[1], 0.0)
        response = position_control(request)
        print(response)






