#!/usr/bin/env python3


import rospy
import math


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

from ca2_ttk4192.srv import positionControl, positionControlResponse

from tf.transformations import euler_from_quaternion

class PosControl():
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.odom = None
        self.position_control = rospy.Service('position_control', positionControl, self.move_to_point)
        self.rate = 0.05

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.kp = 0.1
        self.speed = Twist()
        self.CONSTANT_ANGULAR_SPEED = 5
        self.CONSTANT_LINEAR_SPEED = 5

        self.MAX_LINEAR_SPEED = 0.3
        self.MIN_LINEAR_SPEED = 0.1
        self.DISTANCE_THRESHOLD = 0.05
        self.ORIENTATION_THRESHOLD_LOW = 0.07
        self.ORIENTATION_THRESHOLD_HIGH = 0.3
        self.orientation_threshold = self.ORIENTATION_THRESHOLD_LOW
        

    def odom_callback(self, data):
        self.odom = [data.pose.pose.position, data.pose.pose.orientation]


    def move_to_point(self, request):
        x = request.pos.x
        y = request.pos.y
        self.destination = False

        while (not self.destination):

            robotPosition, currentOrientation = self.odom
            orientation_list = [currentOrientation.x, currentOrientation.y, currentOrientation.z, currentOrientation.w]
            currentOrientation = euler_from_quaternion(orientation_list)[-1]
            errorX = x - robotPosition.x
            errorY = y - robotPosition.y
            self.distance = math.sqrt(errorX**2 + errorY**2)
            print(f"The distance error is {self.distance}")
            self.requiredPathAngle = math.atan2(errorY, errorX)
            self.errorOrientation = self.requiredPathAngle - currentOrientation 
            if abs(self.requiredPathAngle - currentOrientation) > self.orientation_threshold and self.distance > self.DISTANCE_THRESHOLD:
                self.orientation_threshold = self.ORIENTATION_THRESHOLD_LOW
                if self.requiredPathAngle > currentOrientation:
                    self.speed.angular.z = self.kp*abs(self.errorOrientation)*self.CONSTANT_ANGULAR_SPEED
                else:
                    self.speed.angular.z = -self.kp*abs(self.errorOrientation)*self.CONSTANT_ANGULAR_SPEED
                self.speed.linear.x = 0.0
            else:
                self.orientation_threshold = self.ORIENTATION_THRESHOLD_HIGH
                if self.requiredPathAngle > currentOrientation:
                    self.speed.angular.z = 2*self.kp*abs(self.errorOrientation)*self.CONSTANT_ANGULAR_SPEED
                else:
                    self.speed.angular.z = -2*self.kp*abs(self.errorOrientation)*self.CONSTANT_ANGULAR_SPEED
                if self.distance > self.DISTANCE_THRESHOLD:
                    self.speed.linear.x = max(min(0.7*self.speed.linear.x+0.3*self.CONSTANT_LINEAR_SPEED*self.kp*self.distance, self.MAX_LINEAR_SPEED), self.MIN_LINEAR_SPEED)
                else:
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = 0.0
                    self.destination = True
                    print(f"Destination is reached and the final position value is {robotPosition.x}, {robotPosition.y}")
            

            print(f"The speed is {self.speed.linear.x}")
            print(f"The angular speed is {self.speed.angular.z}")
            self.cmd_vel_pub.publish(self.speed)
            rospy.sleep(self.rate)

        return True



rospy.init_node('position_control')

pos_control = PosControl()

while not rospy.is_shutdown():
    rospy.sleep(pos_control.rate)


