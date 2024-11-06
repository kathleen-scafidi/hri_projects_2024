#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math

class LaserScanProcessor:
    def __init__(self):
        
        rospy.init_node('laser_scan_processor', anonymous=True)
        
        rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
        
        rospy.loginfo("Laser scan processor node initialized")

    def scan_callback(self, scan_data):
        """
        Callback function for the laser scan subscriber.
        Processes incoming laser scan data and finds the closest reading.
        
        Args:
            scan_data (LaserScan): Incoming laser scan message
        """
        #initializes variables to store the closest reading
        closest_distance = float('inf')  #start with infinity as the closest value
        closest_angle = 0
        
        #loop through the ranges to find the closest reading
        for i, distance in enumerate(scan_data.ranges):
        
            if not math.isinf(distance) and not math.isnan(distance):
                if distance > 0 and distance < closest_distance:
                    closest_distance = distance
                    #get the angle for the closest reading
                    closest_angle = scan_data.angle_min + (i * scan_data.angle_increment)
        
        #converting angle from radians to degrees for easier reading
        angle_degrees = math.degrees(closest_angle)
        
        rospy.loginfo(f"Closest obstacle: {closest_distance:.2f} meters at angle: {angle_degrees:.2f} degrees")

    def run(self):
        """
        Main run loop for the node.
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        #laser scan processor being created and run
        node = LaserScanProcessor()
        node.run()
    except rospy.ROSInterruptException:
        pass