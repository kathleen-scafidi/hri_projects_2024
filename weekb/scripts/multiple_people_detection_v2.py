#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from sklearn.linear_model import LinearRegression

class FormationDetector:
    def __init__(self):
        rospy.init_node('formation_detector', anonymous=True)
        
        # Parameters for group detection
        self.max_group_distance = 0.8  # meters
        self.min_group_size = 2  # people
        self.line_deviation_threshold = 0.2  # meters
        self.circle_radius_tolerance = 0.2  # meters
        
        # Detection parameters from leg detector
        self.leg_width_min = 0.1
        self.leg_width_max = 0.25
        self.leg_depth_min = 0.05
        self.leg_depth_max = 0.5
        
        # Subscribe to the laser scan data to process legs
        self.laser_subscriber = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
        self.detected_legs = []
        
        rospy.loginfo("Formation detector initialized with following parameters:")
        rospy.loginfo(f"Max group distance: {self.max_group_distance:.2f}m")
        rospy.loginfo(f"Min group size: {self.min_group_size} people")
        rospy.loginfo(f"Line deviation threshold: {self.line_deviation_threshold:.2f}m")
        rospy.loginfo(f"Circle radius tolerance: {self.circle_radius_tolerance:.2f}m")

    def scan_callback(self, scan_data):
        """Process laser scan data to detect legs and identify formations"""
        # First detect legs using the leg detector logic
        self.detected_legs = self.detect_legs(scan_data)
        
        if self.detected_legs:
            rospy.loginfo(f"Detected {len(self.detected_legs)} potential legs")
        
        # If we have enough legs, look for formations
        if len(self.detected_legs) >= self.min_group_size * 2:  # Each person has 2 legs
            groups = self.find_groups(self.detected_legs)
            rospy.loginfo(f"Found {len(groups)} potential groups")
            
            for i, group in enumerate(groups):
                group_size = len(group)
                rospy.loginfo(f"Analyzing group {i+1} with {group_size} people")
                
                if self.is_line_formation(group):
                    rospy.loginfo(f"Line formation detected with {group_size} people!")
                elif self.is_circle_formation(group):
                    rospy.loginfo(f"Circle formation detected with {group_size} people!")
                else:
                    rospy.loginfo(f"Group {i+1} is in an undefined formation")

    def detect_legs(self, scan_data):
        """Detect legs in the laser scan data"""
        detected_legs = []
        segments = []
        current_segment = []
        
        # Convert scanned data into segments
        for i, distance in enumerate(scan_data.ranges):
            if not math.isinf(distance) and not math.isnan(distance):
                if self.leg_depth_min < distance < self.leg_depth_max:
                    angle = scan_data.angle_min + (i * scan_data.angle_increment)
                    x = distance * math.cos(angle)
                    y = distance * math.sin(angle)
                    current_segment.append((x, y, distance))
                elif current_segment:
                    segments.append(current_segment)
                    current_segment = []
                    
        if current_segment:
            segments.append(current_segment)

        rospy.loginfo(f"Processing {len(segments)} potential leg segments")

        # Analyze segments for leg-like characteristics
        for i, segment in enumerate(segments):
            if len(segment) < 2:
                continue
                
            # Calculate segment width
            start_x, start_y, start_dist = segment[0]
            end_x, end_y, end_dist = segment[-1]
            width = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
            
            if self.leg_width_min < width < self.leg_width_max:
                center_x = (start_x + end_x) / 2
                center_y = (start_y + end_y) / 2
                detected_legs.append((center_x, center_y))
                rospy.loginfo(f"Leg detected! Width: {width:.2f}m, Distance: {start_dist:.2f}m")
                
        return detected_legs

    def find_groups(self, legs):
        """Group legs into potential people and then into groups"""
        groups = []
        unassigned_legs = legs.copy()
        people_positions = []
        
        # First pair legs into people
        while len(unassigned_legs) >= 2:
            leg1 = unassigned_legs[0]
            closest_leg = None
            min_distance = float('inf')
            
            for leg2 in unassigned_legs[1:]:
                dist = math.sqrt((leg1[0] - leg2[0])**2 + (leg1[1] - leg2[1])**2)
                if dist < min_distance and dist < self.leg_width_max:
                    min_distance = dist
                    closest_leg = leg2
            
            if closest_leg:
                person_x = (leg1[0] + closest_leg[0]) / 2
                person_y = (leg1[1] + closest_leg[1]) / 2
                people_positions.append((person_x, person_y))
                rospy.loginfo(f"Paired legs at distance {min_distance:.2f}m apart")
                unassigned_legs.remove(leg1)
                unassigned_legs.remove(closest_leg)
            else:
                unassigned_legs.remove(leg1)
                rospy.loginfo("Found unpaired leg - removing from analysis")

        rospy.loginfo(f"Detected {len(people_positions)} people from leg pairs")

        # Now group people based on proximity
        unassigned_people = people_positions.copy()
        while unassigned_people:
            current_group = []
            current_person = unassigned_people[0]
            current_group.append(current_person)
            unassigned_people.remove(current_person)
            
            # Find all people within max_group_distance
            i = 0
            while i < len(unassigned_people):
                person = unassigned_people[i]
                dist = math.sqrt((current_person[0] - person[0])**2 + 
                               (current_person[1] - person[1])**2)
                if dist <= self.max_group_distance:
                    current_group.append(person)
                    rospy.loginfo(f"Added person to group at distance {dist:.2f}m")
                    unassigned_people.remove(person)
                else:
                    i += 1
            
            if len(current_group) >= self.min_group_size:
                groups.append(current_group)
                rospy.loginfo(f"Formed new group with {len(current_group)} people")
                
        return groups

    def is_line_formation(self, group):
        """Check if group is arranged in a line"""
        if len(group) < self.min_group_size:
            return False
            
        X = np.array([[p[0]] for p in group])
        Y = np.array([p[1] for p in group])
        
        reg = LinearRegression().fit(X, Y)
        y_pred = reg.predict(X)
        avg_deviation = np.mean(np.abs(Y - y_pred))
        
        rospy.loginfo(f"Line formation analysis - Average deviation: {avg_deviation:.3f}m")
        return avg_deviation < self.line_deviation_threshold

    def is_circle_formation(self, group):
        """Check if group is arranged in a circle"""
        if len(group) < 3:  # Need at least 3 people for a circle
            return False
            
        # Calculate center point
        center_x = sum(p[0] for p in group) / len(group)
        center_y = sum(p[1] for p in group) / len(group)
        
        # Check if all points are roughly equidistant from center
        distances = [math.sqrt((p[0] - center_x)**2 + (p[1] - center_y)**2) 
                    for p in group]
        avg_radius = sum(distances) / len(distances)
        
        max_deviation = max(abs(d - avg_radius) for d in distances)
        rospy.loginfo(f"Circle formation analysis - Average radius: {avg_radius:.2f}m, Max deviation: {max_deviation:.3f}m")
        return max_deviation < self.circle_radius_tolerance

    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("Formation detector running at 10 Hz")
        while not rospy.is_shutdown():
            rate.sleep()

def main():
    try:
        detector = FormationDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Formation detector stopped by user")
        pass

if __name__ == '__main__':
    main()