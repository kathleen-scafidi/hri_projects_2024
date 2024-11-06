#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math


class Shapes:
    def __init__(self):
        
        rospy.init_node('shapes', anonymous=True)
        
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
      
        self.rate = rospy.Rate(10)  
       
        self.vel_msg = Twist()
        
        #how fast to move and turn
        self.linear_speed = 0.8  
        self.angular_speed = 0.8  

    def stop(self):
        """Stop the robot"""
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
        rospy.sleep(1)

    def move_straight(self, distance):
        """Move the robot in a straight line"""
        #calculate time needed to move distance
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #move forward
        self.vel_msg.linear.x = self.linear_speed
        self.vel_msg.angular.z = 0

        #move until distance is reached
        while current_distance < distance:
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = self.linear_speed * (t1 - t0)
            self.rate.sleep()

        self.stop()

    def rotate(self, angle_degrees):
        """Rotate the robot by a specified angle in degrees"""
        angle_radians = math.radians(angle_degrees)
        
        #time it takes to rotate
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        #rotation speed
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = self.angular_speed

        #rotate until angle is met
        while current_angle < abs(angle_radians):
            if angle_degrees < 0:  # Rotate clockwise
                self.vel_msg.angular.z = -abs(self.angular_speed)
            else:  # Rotate counter-clockwise
                self.vel_msg.angular.z = abs(self.angular_speed)
                
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.angular_speed * (t1 - t0)
            self.rate.sleep()

        self.stop()

    def square(self, side_length=1.0):
        """Move the robot in a square"""
        rospy.loginfo("Moving in a square...")
        for _ in range(4):
            self.move_straight(side_length)
            self.rotate(90)  
        rospy.loginfo("Square completed")

    def figure_eight(self, radius=1.0):
        """Move the robot in a figure-eight"""
        rospy.loginfo("Moving in a figure-eight...")
        
        for direction in [1, -1]:  #counter-clockwise is 1, clockwise is -1
            #circle size
            circumference = 2 * math.pi * radius
            duration = circumference / self.linear_speed
            angular_speed = (2 * math.pi) / duration

            #velocity of circles
            self.vel_msg.linear.x = self.linear_speed
            self.vel_msg.angular.z = direction * angular_speed

            #move in circles
            t0 = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - t0) < duration:
                self.velocity_publisher.publish(self.vel_msg)
                self.rate.sleep()

        self.stop()
        rospy.loginfo("Figure-eight completed")

    def triangle(self, side_length=1.0):
        """Move the robot in a triangular"""
        rospy.loginfo("Moving in a triangular...")
        for _ in range(3):
            self.move_straight(side_length)
            self.rotate(120)  
        rospy.loginfo("Triangle completed")

def main():
    try:
        #shapes object
        mover = Shapes()
        rospy.sleep(1)  

        #allow user to choose which shape to make
        choice = rospy.get_param('~shape', 'square')  
        
        if choice == 'square':
            mover.square()
        elif choice == 'figure_eight':
            mover.figure_eight()
        elif choice == 'triangle':
            mover.triangle()
        else:
            rospy.loginfo("Invalid choice. Choose 'square', 'figure_eight', or 'triangle'")

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()