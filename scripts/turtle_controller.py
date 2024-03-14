#!/usr/bin/env python3

from turtlesim.srv import Spawn
from std_srvs.srv import Empty
from turtlesim.srv import Kill
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math

class PID:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        self.integral += error
        derivative = (error - self.prev_error)
        self.prev_error = error
        return self.P * error + self.I * self.integral + self.D * derivative


class TurtleController:

    def __init__(self):
        velocity_topic = '/turtle1/cmd_vel'
        pose_topic = '/turtle1/pose'
        self.velocity_publisher = rospy.Publisher(velocity_topic, Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber(pose_topic, Pose, self.update_pose_cb)
        self.pose = Pose()
        self.rate = rospy.Rate(100)


    def go_to_goal(self, x_goal, y_goal):

        velocity_message = Twist()
        threshold = 0.01

        linear_PID = PID(1, 0.001, 0.1)
        angular_PID = PID(50, 0.001, 3)

        while True:

            distance = self.ecluidean_distance(x_goal, y_goal)
            linear_speed = linear_PID.update(distance)

            desired_angle = math.atan2(y_goal - self.pose.y, x_goal - self.pose.x)
            angular_speed = angular_PID.update(desired_angle - self.pose.theta)

            velocity_message.linear.x = linear_speed
            velocity_message.angular.z = angular_speed
            self.velocity_publisher.publish(velocity_message)

            self.rate.sleep()

            if distance < threshold:
                print("Goal reached")
                break

        return True


    def ecluidean_distance(self, x, y):
        '''Returns the euclidean distance between the current position and the goal position'''
        return math.sqrt((x - self.pose.x)**2 + (y - self.pose.y)**2)


    # killing the existing turtle and spawning the turtle at the given position
    def spawn_turtle(self, x, y, theta):
        rospy.wait_for_service('spawn')
        rospy.wait_for_service('kill')
        try:
            killer = rospy.ServiceProxy('kill', Kill)
            killer('turtle1')
            spawner = rospy.ServiceProxy('spawn', Spawn)
            response = spawner(x, y, theta, 'turtle1')
            print(f"Turtle {response.name} respawned at position ({x}, {y})")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def clear_screen(self):
        rospy.wait_for_service('clear')
        try:
            clear = rospy.ServiceProxy('clear', Empty)
            response = clear()
            print(f"Screen cleared")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            
    def update_pose_cb(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)


