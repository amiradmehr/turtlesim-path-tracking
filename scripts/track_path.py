#!/usr/bin/env python3

import rospy
import sys
from turtle_controller import TurtleController
from extract_path import coordinates
if __name__ == '__main__':

    try:
        rospy.init_node('turtle_controller', anonymous=False)
        controller = TurtleController()

        # spawn the turtle at (1,1)
        controller.spawn_turtle(1,1,0)

        # clear the screen
        controller.clear_screen()

        # importing the coordinates from the extract_path.py
        M_coordinates = coordinates(file='edges.csv')
        points = M_coordinates.read_csv()
        points = M_coordinates.normalize_coordinates_to_window(points, window_width=10, window_height=9, offset=(1,1))
        
        # go to each point in the points array

        for point in points:
            reached = controller.go_to_goal(point[0], point[1])
            rospy.loginfo(f'Arrived at {point}')

            # wait for turtle to reach the goal
            while not reached:
                reached = controller.go_to_goal(point[0], point[1])
                rospy.loginfo(f'Arrived at {point}')
                rospy.sleep(1)

        rospy.loginfo('Path completed')
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")