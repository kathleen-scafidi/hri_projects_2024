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
        self.is_moving = False
        
        #detecting obstacles and legs
        self.min_safe_distance = 0.5 #safe distance
        self.leg_width_min = 0.1  #min leg width (generic)
        self.leg_width_max = 0.25  #max leg width (generic)
        self.leg_depth_min = 0.05  #min distance from leg
        self.leg_depth_max = 0.5  # max distance to find leg
        self.leg_pair_max_distance = 0.4  #distance between pairs of legs
        self.legs_detected = False
        
        self.closest_distance = float('inf')
        self.closest_angle = 0
        self.laser_data = None

        rospy.loginfo("Obstacle avoider with leg detection")

    def scan_callback(self, scan_data):
        """
        Callback function for the laser scan subscriber.
        Processes incoming laser scan data, finds the closest reading, and checks for legs.
        """
        self.laser_data = scan_data
        self.closest_distance = float('inf')
        
        #first check for legs
        self.legs_detected = self.detect_legs(scan_data)
        
        if not self.legs_detected:
            #move on to obstacle detection if legs are not detected
            for i, distance in enumerate(scan_data.ranges):
                if not math.isinf(distance) and not math.isnan(distance):
                    #ignore any of the readings that could be legs
                    if self.leg_depth_min <= distance <= self.leg_depth_max:
                        continue
                    if distance > 0 and distance < self.closest_distance:
                        self.closest_distance = distance
                        self.closest_angle = scan_data.angle_min + (i * scan_data.angle_increment)
            
            angle_degrees = math.degrees(self.closest_angle)
            rospy.loginfo(f"Closest obstacle: {self.closest_distance:.2f} meters at angle: {angle_degrees:.2f} degrees")
        else:
            rospy.loginfo("I found someone! I'll stop here.")

    def detect_legs(self, scan_data):
        """
        Detect legs in the laser scan data based on width and depth characteristics.
        Returns: True if at least one leg is detected, False otherwise
        """
        if not scan_data:
            return False

        #convert scanned data into segments for easier reading
        segments = []
        current_segment = []
        
        for i, distance in enumerate(scan_data.ranges):
            if not math.isinf(distance) and not math.isnan(distance):
                if self.leg_depth_min < distance < self.leg_depth_max:
                    current_segment.append((i, distance))
                elif current_segment:
                    segments.append(current_segment)
                    current_segment = []
        
        if current_segment:
            segments.append(current_segment)

        #look through segments for leg-like features
        for segment in segments:
            if len(segment) < 2:
                continue
                
            #get segment width
            start_idx, start_dist = segment[0]
            end_idx, end_dist = segment[-1]
            angle_width = (end_idx - start_idx) * scan_data.angle_increment
            width = math.sqrt(start_dist**2 + end_dist**2 - 
                            2 * start_dist * end_dist * math.cos(angle_width))
            
            #if we find a segment that seems like a leg, return true
            if self.leg_width_min < width < self.leg_width_max:
                rospy.loginfo(f"Leg detected! Width: {width:.2f}m, Distance: {start_dist:.2f}m")
                return True
        
        return False

    def find_clear_direction(self):
        """
        Find a clear direction to move by analyzing laser scan data
        Returns: angle to turn (in degrees), or None if no clear path is found
        """
        if not self.laser_data:
            return None
            
        front_angle_range = math.pi/4  
        num_readings = len(self.laser_data.ranges)
        center_idx = num_readings // 2
        sector_size = int((front_angle_range / (self.laser_data.angle_max - self.laser_data.angle_min)) * num_readings)
        
        for direction in ['right', 'left']:
            clear_path = True
            if direction == 'right':
                start_idx = 0
                end_idx = sector_size
            else:
                start_idx = num_readings - sector_size
                end_idx = num_readings
                
            for i in range(start_idx, end_idx):
                distance = self.laser_data.ranges[i]
                if not math.isinf(distance) and not math.isnan(distance):
                    if distance < self.min_safe_distance:
                        clear_path = False
                        break
                        
            if clear_path:
                return -90 if direction == 'right' else 90
                
        return 180

    def avoid_obstacle(self):
        """
        When an obstacle is detected, stop, find a clear direction, and turn that way
        Returns: True if a clear path was found and robot turns, False otherwise
        """
        rospy.loginfo("Avoiding obstacle...")
        self.stop()
        
        turn_angle = self.find_clear_direction()
        if turn_angle is None:
            rospy.loginfo("No clear path found!")
            return False
            
        rospy.loginfo(f"Turning {turn_angle} degrees to avoid obstacle")
        self.rotate(turn_angle)
        return True

    def is_obstacle_ahead(self):
        """Check if there's an obstacle too close ahead, excluding potential legs"""
        if not self.laser_data:
            return False
            
        center_idx = len(self.laser_data.ranges) // 2
        sector_size = len(self.laser_data.ranges) // 6  
        
        for i in range(center_idx - sector_size, center_idx + sector_size):
            distance = self.laser_data.ranges[i]
            if not math.isinf(distance) and not math.isnan(distance):
                #skip any readings that could be legs
                if self.leg_depth_min <= distance <= self.leg_depth_max:
                    continue
                if distance < self.min_safe_distance:
                    return True
        return False

    def stop(self):
        """Stop the robot"""
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
        self.is_moving = False
        rospy.sleep(1)

    def rotate(self, angle_degrees):
        """Rotate the robot by a specific angle in degrees"""
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

    def start_moving(self):
        """Start moving forward"""
        self.vel_msg.linear.x = self.linear_speed
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
        self.is_moving = True

    def navigate_continuously(self):
        """
        Continuously navigate and avoid obstacles until legs are detected
        """
        rospy.loginfo("Starting continuous navigation...")
        self.start_moving()

        try:
            while not rospy.is_shutdown():
                if self.legs_detected:
                    rospy.loginfo("Legs detected - stopping navigation")
                    self.stop()
                    break
                    
                if self.is_obstacle_ahead():
                    rospy.loginfo("Obstacle detected ahead!")
                    
                    if self.avoid_obstacle():
                        rospy.loginfo("Successfully avoided obstacle, resuming movement...")
                        self.start_moving()
                    else:
                        rospy.loginfo("Searching for alternative path...")
                        #try different angles until a clear path is found
                        for angle in [45, -45, 90, -90, 135, -135, 180]:
                            self.rotate(angle)
                            if not self.is_obstacle_ahead():
                                rospy.loginfo(f"Found clear path after rotating {angle} degrees")
                                self.start_moving()
                                break
                        else:
                            rospy.loginfo("No clear path found in any direction!")
                            rospy.sleep(2)
                            continue
                
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
        rospy.sleep(1)  #time for initialization
        
        #start
        avoider.navigate_continuously()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()