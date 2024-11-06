#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObstacleAvoider:
    def __init__(self):
        rospy.init_node('obstacle_avoider', anonymous=True)
        
        #publishers and subscribers
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
      
        self.rate = rospy.Rate(10)  
        self.vel_msg = Twist()
        
        #movement
        self.linear_speed = 0.8  
        self.angular_speed = 0.8
        
        #detecting obstacles
        self.min_safe_distance = 0.5  #set minimum safe distance
        self.closest_distance = float('inf')
        self.closest_angle = 0
        self.laser_data = None

        rospy.loginfo("Obstacle avoider initialized with laser scan processing")

    def scan_callback(self, scan_data):
        """
        Callback function for the laser scan subscriber.
        Processes incoming laser scan data and finds the closest reading.
        """
        self.laser_data = scan_data
        self.closest_distance = float('inf')
        
        #loop to get the closest reading
        for i, distance in enumerate(scan_data.ranges):
            if not math.isinf(distance) and not math.isnan(distance):
                if distance > 0 and distance < self.closest_distance:
                    self.closest_distance = distance
                    #get the angle 
                    self.closest_angle = scan_data.angle_min + (i * scan_data.angle_increment)
        
        #convert to radians
        angle_degrees = math.degrees(self.closest_angle)
        rospy.loginfo(f"Closest obstacle: {self.closest_distance:.2f} meters at angle: {angle_degrees:.2f} degrees")

    def find_clear_direction(self):
        """
        Find a clear direction to move by analyzing laser scan data
        Returns: angle to turn (in degrees), or None if no clear path found
        """
        if not self.laser_data:
            return None
            
        #set the "blocking" distance as 45 degrees around robot
        front_angle_range = math.pi/4  
        
        #number of readings we got from the scan
        num_readings = len(self.laser_data.ranges)
        
        
        center_idx = num_readings // 2
        sector_size = int((front_angle_range / (self.laser_data.angle_max - self.laser_data.angle_min)) * num_readings)
        
        #look for clear paths in different directions
        for direction in ['right', 'left']:
            clear_path = True
            if direction == 'right':
                start_idx = 0
                end_idx = sector_size
            else:
                start_idx = num_readings - sector_size
                end_idx = num_readings
                
            #make sure it's clear in front 
            for i in range(start_idx, end_idx):
                distance = self.laser_data.ranges[i]
                if not math.isinf(distance) and not math.isnan(distance):
                    if distance < self.min_safe_distance:
                        clear_path = False
                        break
                        
            if clear_path:
                #if the path is clear, which way to go
                return -90 if direction == 'right' else 90
                
        #if there is no clear path keep turning
        return 180

    def avoid_obstacle(self):
        """
        When an obstacle is detected, stop, find a clear direction, and turn that way
        Returns: True if a clear path was found and robot turned, False otherwise
        """
        rospy.loginfo("Avoiding obstacle...")
        self.stop()
        
        #find which way to turn
        turn_angle = self.find_clear_direction()
        if turn_angle is None:
            rospy.loginfo("No clear path found!")
            return False
            
        rospy.loginfo(f"Turning {turn_angle} degrees to avoid obstacle")
        self.rotate(turn_angle)
        return True

    def is_obstacle_ahead(self):
        """Check if there's an obstacle too close ahead"""
        if not self.laser_data:
            return False
            
        #check the middle of the scan for obstacles
        center_idx = len(self.laser_data.ranges) // 2
        sector_size = len(self.laser_data.ranges) // 6  
        
        for i in range(center_idx - sector_size, center_idx + sector_size):
            distance = self.laser_data.ranges[i]
            if not math.isinf(distance) and not math.isnan(distance):
                if distance < self.min_safe_distance:
                    return True
        return False

    def stop(self):
        """Stop the robot"""
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
        rospy.sleep(1)

    def rotate(self, angle_degrees):
        """Rotate the robot by a specified angle in degrees"""
        angle_radians = math.radians(angle_degrees)
        
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = self.angular_speed

        while current_angle < abs(angle_radians) and not rospy.is_shutdown():
            if angle_degrees < 0:  #rotate clockwise
                self.vel_msg.angular.z = -abs(self.angular_speed)
            else:  #rotate counter-clockwise
                self.vel_msg.angular.z = abs(self.angular_speed)
                
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.angular_speed * (t1 - t0)
            self.rate.sleep()

        self.stop()

    def move_forward(self):
        """
        Continuously move forward while avoiding obstacles
        """
        self.vel_msg.linear.x = self.linear_speed
        self.vel_msg.angular.z = 0

        while not rospy.is_shutdown():
            if self.is_obstacle_ahead():
                rospy.loginfo("Obstacle detected ahead!")
                if not self.avoid_obstacle():
                    self.stop()
                    rospy.loginfo("Cannot find clear path, stopping...")
                    return
            
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()

    def start_moving(self):
        """Start moving forward"""
        self.vel_msg.linear.x = self.linear_speed
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
        self.is_moving = True


    def navigate_continuously(self):
        """
        Continuously navigate and avoid obstacles, always trying to keep moving
        """
        rospy.loginfo("Starting continuous navigation...")
        self.start_moving()

        try:
            while not rospy.is_shutdown():
                if self.is_obstacle_ahead():
                    rospy.loginfo("Obstacle detected ahead!")
                    
                    if self.avoid_obstacle():
                        rospy.loginfo("Successfully avoided obstacle, resuming movement...")
                        self.start_moving()
                    else:
                        rospy.loginfo("Searching for alternative path...")
                        #rotating in increments until a clear path is found
                        for angle in [45, -45, 90, -90, 135, -135, 180]:
                            self.rotate(angle)
                            if not self.is_obstacle_ahead():
                                rospy.loginfo(f"Found clear path after rotating {angle} degrees")
                                self.start_moving()
                                break
                        else:
                            rospy.loginfo("No clear path found in any direction!")
                            #wait a bit and try again
                            rospy.sleep(2)
                            continue
                
                #if we're not moving, start moving
                if not self.is_moving:
                    self.start_moving()
                
                self.velocity_publisher.publish(self.vel_msg)
                self.rate.sleep()

        except rospy.ROSInterruptException:
            self.stop()
            rospy.loginfo("Navigation stopped by user")



def main():
    try:
        avoider = ObstacleAvoider()
        rospy.sleep(1)  
        
        avoider.move_forward()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
    