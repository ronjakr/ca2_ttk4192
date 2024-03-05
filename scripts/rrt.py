#!/usr/bin/env python3

import rospy
from collision_detection import distance

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ca2_ttk4192.srv import isThroughObstacle, isThroughObstacleRequest, isInObstacle, isInObstacleRequest, positionControl, positionControlRequest

class Node: # maybe create dictionary instead in order to create list of positions?
    def __init__(self, position, parent) -> None:
        self.node = position
        self.parent = parent


def random_configuration():
    """
    Samples a random point in space
    """
    pass


def nearest_vertex(q_random, nodes):
    """
    Finding the node that is closest to q_random
    """
    nearest_vertex = min(nodes, key=lambda node: distance(q_random, node.position))
    return nearest_vertex


def new_configuration(q_nearest, q_random, delta_q):
    """
    Scaling distance between q_nearest and q_random to be
    maximum equal to delta_q
    """
    distance = distance(q_nearest, q_random)

    if distance > delta_q:
        q_new = Point(q_new[0] + delta_q, q_new[1] + delta_q)  # TODO this is wrong, make it correct

    return q_new


def rrt_planner(q_init, q_end, K, delta_q) -> MarkerArray:  # K: number of vertices in RRT
    obstacle_radius = 0.3
    nodes = []   # "tree"/nodes/G
    nodes.append(Node(position=q_init, parent=None))

    for k in range(K):
        q_random = random_configuration()   # a random point in space

        i=0
        while not isInObstacle(q_random,obstacle_radius):   # trying to sample a point in obstacle-free space
            q_random = random_configuration()   # a random point in space
            i+=1
            if i == 10:
                break

        q_nearest = nearest_vertex(q_random, nodes)    # point in "tree" that is closest to the random point
        q_new = new_configuration(q_nearest, q_random, delta_q)    # modifying configuration in case q_random is too far from q_nearest
        if not (isInObstacle(q_new, obstacle_radius) and isThruObstacle(q_new, q_nearest, obstacle_radius)):
            nodes.append(Node(position=q_new, parent=q_nearest))

    nodes.append(Node(position=q_end, parent=q_new))
    return nodes


def visualize_path(tree):
    pass



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
    incremental_distance = 1
    num_vertices = 100 # nodes

    G = rrt_planner(q_init=startpos, q_end=endpos, K=num_vertices, delta_q=incremental_distance)



    tree_publisher = rospy.Publisher('tree_marker', MarkerArray, queue_size=10)
    tree_marker = MarkerArray()
    marker_identity = 0

    # Create a blue-green square representing the start
    start_rgb_color = [0/256, 158/256, 115/256]
    start_marker_size = 0.2
    start_marker = get_marker(Marker.CUBE, G[-1].position, start_marker_size, start_rgb_color, marker_identity)
    marker_identity += 1

    # Create a vermillion square representing the goal
    end_rgb_color = [213/256, 94/256, 0/256]
    end_marker_size = 0.2
    end_marker = get_marker(Marker.CUBE, G[1].position, end_marker_size, end_rgb_color, marker_identity)
    marker_identity +=1

    tree_marker.markers.append(start_marker)
    tree_marker.markers.append(end_marker)

    
    # Create reddish purple edges 
    edge_color = [204/256, 121/256, 167/256]
    
    for index in range(len(G)-1):
        first_point = G[index].node
        second_point = G[index+1].parent
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


    list_of_position = [[0.0, 2.0], [1.0, 2.0], [1.0, 1.0]] # change to actual list of points by traversing from child to parent
    position_control = rospy.ServiceProxy('/position_control', positionControl)

    for position in list_of_position:
        rospy.wait_for_service('/position_control')
        request = Point(position[0], position[1], 0.0)
        response = position_control(request)
        print(response)






