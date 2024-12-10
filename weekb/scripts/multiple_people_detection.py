#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import LaserScan
from sklearn.linear_model import LinearRegression
import tf2_ros
import tf2_geometry_msgs

class FormationDetector:
    """
    A ROS node that detects human formations using laser scan data.
    This detector can identify line and circle formations by analyzing leg positions.
    """
    
    def __init__(self):
        """Initialize the FormationDetector node with all necessary parameters and publishers."""
        # Initialize the ROS node
        rospy.init_node('formation_detector', anonymous=True)
        
        # Create publishers for debug visualization
        self.leg_marker_pub = rospy.Publisher('detected_legs', PointStamped, queue_size=10)
        self.debug_enabled = rospy.get_param('~debug', True)
        
        # Load all parameters from ROS parameter server with defaults
        self.load_parameters()
        
        # Set up TF2 for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Verify that required topics exist before subscribing
        self.verify_topics()
        
        # Subscribe to laser scan data
        self.laser_subscriber = rospy.Subscriber('/robot_0/base_scan', LaserScan, self.scan_callback)
        self.detected_legs = []
        
        # Log initialization and parameters
        rospy.loginfo("Formation detector initialized with following parameters:")
        self.log_parameters()
        
    def detect_legs(self, scan_data):
        """
        Detect legs in the laser scan data by identifying leg-like segments.
        Args:
            scan_data: LaserScan message containing range data
        Returns:
            List of (x, y) tuples representing detected leg positions
        """
        detected_legs = []
        segments = []
        current_segment = []
        
        try:
            # Convert scan data into segments
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

            # Analyze segments for leg characteristics
            for segment in segments:
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
                    
                    # Publish debug visualization
                    self.publish_debug_marker(center_x, center_y)
                    rospy.loginfo(f"Leg detected! Width: {width:.2f}m, Distance: {start_dist:.2f}m")
                    
            return detected_legs
            
        except Exception as e:
            rospy.logerr(f"Error in leg detection: {e}")
            return []
        
    def publish_debug_marker(self, x, y):
        """Publish visualization markers for detected legs."""
        if not self.debug_enabled:
            return
            
        try:
            point = PointStamped()
            point.header.frame_id = "base_link"
            point.header.stamp = rospy.Time.now()
            point.point.x = x
            point.point.y = y
            point.point.z = 0
            
            self.leg_marker_pub.publish(point)
        except Exception as e:
            rospy.logwarn(f"Failed to publish debug marker: {e}")

    def load_parameters(self):
        """Load all parameters from ROS parameter server with default values."""
        # Group detection parameters
        self.max_group_distance = rospy.get_param('~max_group_distance', 0.8)
        self.min_group_size = rospy.get_param('~min_group_size', 2)
        self.line_deviation_threshold = rospy.get_param('~line_deviation_threshold', 0.2)
        self.circle_radius_tolerance = rospy.get_param('~circle_radius_tolerance', 0.2)
        
        # Leg detection parameters
        self.leg_width_min = rospy.get_param('~leg_width_min', 0.1)
        self.leg_width_max = rospy.get_param('~leg_width_max', 0.25)
        self.leg_depth_min = rospy.get_param('~leg_depth_min', 0.05)
        self.leg_depth_max = rospy.get_param('~leg_depth_max', 0.5)

    def verify_topics(self):
        """
        Verify that all required ROS topics are available.
        Raises ROSException if required topics are not found.
        """
        try:
            topics = rospy.get_published_topics()
            scan_topics = [topic for topic, type_name in topics if 'LaserScan' in type_name]
            
            if not scan_topics:
                rospy.logwarn("No LaserScan topics found! Available topics:")
                for topic, type_name in topics:
                    rospy.logwarn(f"  {topic}: {type_name}")
                raise rospy.ROSException("Required LaserScan topic not found")
                
            # if '/base_scan' not in scan_topics:
            #     rospy.logwarn(f"'/base_scan' not found. Available scan topics: {scan_topics}")
            #     rospy.logwarn("Please update the code to use one of the available topics")
            #     raise rospy.ROSException("Configured topic not found")
        except Exception as e:
            rospy.logerr(f"Error verifying topics: {e}")
            raise

    def log_parameters(self):
        """Log all current parameter values for debugging purposes."""
        params = {
            'max_group_distance': self.max_group_distance,
            'min_group_size': self.min_group_size,
            'line_deviation_threshold': self.line_deviation_threshold,
            'circle_radius_tolerance': self.circle_radius_tolerance,
            'leg_width_min': self.leg_width_min,
            'leg_width_max': self.leg_width_max,
            'leg_depth_min': self.leg_depth_min,
            'leg_depth_max': self.leg_depth_max
        }
        
        for param, value in params.items():
            rospy.loginfo(f"{param}: {value}")

    def scan_callback(self, scan_data):
        """
        Process incoming laser scan data to detect legs and identify formations.
        Args:
            scan_data: LaserScan message containing range data
        """
        try:
            # Detect legs from laser scan
            self.detected_legs = self.detect_legs(scan_data)
            
            if self.detected_legs:
                rospy.loginfo(f"Detected {len(self.detected_legs)} potential legs")
            
            # Analyze formations if enough legs are detected
            if len(self.detected_legs) >= self.min_group_size * 2:
                groups = self.find_groups(self.detected_legs)
                rospy.loginfo(f"Found {len(groups)} potential groups")
                
                # Analyze each group for formations
                for i, group in enumerate(groups):
                    group_size = len(group)
                    rospy.loginfo(f"Analyzing group {i+1} with {group_size} people")
                    
                    if self.is_line_formation(group):
                        rospy.loginfo(f"Line formation detected with {group_size} people!")
                    elif self.is_circle_formation(group):
                        rospy.loginfo(f"Circle formation detected with {group_size} people!")
                    else:
                        rospy.loginfo(f"Group {i+1} is in an undefined formation")
                        
        except Exception as e:
            rospy.logerr(f"Error in scan callback: {e}")

    def find_groups(self, legs):
        """
        Group detected legs into potential people and then into groups.
        Args:
            legs: List of (x, y) tuples representing detected leg positions
        Returns:
            List of groups, where each group is a list of (x, y) positions
        """
        try:
            groups = []
            unassigned_legs = legs.copy()
            people_positions = []
            
            # First pair legs into people
            while len(unassigned_legs) >= 2:
                leg1 = unassigned_legs[0]
                closest_leg = None
                min_distance = float('inf')
                
                # Find closest leg that could form a pair
                for leg2 in unassigned_legs[1:]:
                    dist = math.sqrt((leg1[0] - leg2[0])**2 + (leg1[1] - leg2[1])**2)
                    if dist < min_distance and dist < self.leg_width_max * 2:
                        min_distance = dist
                        closest_leg = leg2
                
                if closest_leg:
                    # Calculate person position as middle point between legs
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

            # Group people based on proximity
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
            
        except Exception as e:
            rospy.logerr(f"Error in group formation: {e}")
            return []

    def is_line_formation(self, group):
        """
        Check if a group is arranged in a line formation using linear regression.
        Args:
            group: List of (x, y) positions representing people in the group
        Returns:
            bool: True if the group forms a line, False otherwise
        """
        try:
            if len(group) < self.min_group_size:
                return False
                
            X = np.array([[p[0]] for p in group])
            Y = np.array([p[1] for p in group])
            
            reg = LinearRegression().fit(X, Y)
            y_pred = reg.predict(X)
            avg_deviation = np.mean(np.abs(Y - y_pred))
            
            rospy.loginfo(f"Line formation analysis - Average deviation: {avg_deviation:.3f}m")
            return avg_deviation < self.line_deviation_threshold
            
        except Exception as e:
            rospy.logerr(f"Error in line formation detection: {e}")
            return False

    def is_circle_formation(self, group):
        """
        Check if a group is arranged in a circle formation.
        Args:
            group: List of (x, y) positions representing people in the group
        Returns:
            bool: True if the group forms a circle, False otherwise
        """
        try:
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
            rospy.loginfo(f"Circle formation analysis - Average radius: {avg_radius:.2f}m, "
                         f"Max deviation: {max_deviation:.3f}m")
            return max_deviation < self.circle_radius_tolerance
            
        except Exception as e:
            rospy.logerr(f"Error in circle formation detection: {e}")
            return False

    def run(self):
        """Main run loop for the formation detector."""
        try:
            rate = rospy.Rate(10)  # 10 Hz
            rospy.loginfo("Formation detector running at 10 Hz")
            while not rospy.is_shutdown():
                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Formation detector interrupted")
        except Exception as e:
            rospy.logerr(f"Error in main loop: {e}")

def main():
    """Main entry point for the formation detector node."""
    try:
        detector = FormationDetector()
        detector.run()
    except rospy.ROSException as e:
        rospy.logerr(f"Failed to start formation detector: {e}")
    except rospy.ROSInterruptException:
        rospy.loginfo("Formation detector stopped by user")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3

# import rospy
# import math
# import numpy as np
# from geometry_msgs.msg import Point, PointStamped
# from sensor_msgs.msg import LaserScan
# from sklearn.linear_model import LinearRegression
# import tf2_ros
# import tf2_geometry_msgs

# class FormationDetector:
#     """
#     A ROS node that detects human formations using laser scan data.
#     This detector can identify line and circle formations by analyzing leg positions.
#     """
    
#     def __init__(self):
#         """Initialize the FormationDetector node with all necessary parameters and publishers."""
#         # Initialize the ROS node
#         rospy.init_node('formation_detector', anonymous=True)
        
#         # Create publishers for debug visualization
#         self.leg_marker_pub = rospy.Publisher('detected_legs', PointStamped, queue_size=10)
#         self.debug_enabled = rospy.get_param('~debug', True)
        
#         # Load all parameters from ROS parameter server with defaults
#         self.load_parameters()
        
#         # Set up TF2 for coordinate transformations
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
#         # Verify that required topics exist before subscribing
#         self.verify_topics()
        
#         # Subscribe to laser scan data
#         self.laser_subscriber = rospy.Subscriber('/robot_0/base_scan', LaserScan, self.scan_callback)
#         self.detected_legs = []
        
#         # Log initialization and parameters
#         rospy.loginfo("Formation detector initialized with following parameters:")
#         self.log_parameters()

#     def load_parameters(self):
#         """Load all parameters from ROS parameter server with default values."""
#         # Group detection parameters
#         self.max_group_distance = rospy.get_param('~max_group_distance', 0.8)
#         self.min_group_size = rospy.get_param('~min_group_size', 2)
#         self.line_deviation_threshold = rospy.get_param('~line_deviation_threshold', 0.2)
#         self.circle_radius_tolerance = rospy.get_param('~circle_radius_tolerance', 0.2)
        
#         # Leg detection parameters
#         self.leg_width_min = rospy.get_param('~leg_width_min', 0.1)
#         self.leg_width_max = rospy.get_param('~leg_width_max', 0.25)
#         self.leg_depth_min = rospy.get_param('~leg_depth_min', 0.05)
#         self.leg_depth_max = rospy.get_param('~leg_depth_max', 0.5)

#     def verify_topics(self):
#         """
#         Verify that all required ROS topics are available.
#         Raises ROSException if required topics are not found.
#         """
#         try:
#             topics = rospy.get_published_topics()
#             scan_topics = [topic for topic, type_name in topics if 'LaserScan' in type_name]
            
#             if not scan_topics:
#                 rospy.logwarn("No LaserScan topics found! Available topics:")
#                 for topic, type_name in topics:
#                     rospy.logwarn(f"  {topic}: {type_name}")
#                 raise rospy.ROSException("Required LaserScan topic not found")
                
#             # if '/base_scan' not in scan_topics:
#             #     rospy.logwarn(f"'/base_scan' not found. Available scan topics: {scan_topics}")
#             #     rospy.logwarn("Please update the code to use one of the available topics")
#             #     raise rospy.ROSException("Configured topic not found")
#         except Exception as e:
#             rospy.logerr(f"Error verifying topics: {e}")
#             raise

#     def log_parameters(self):
#         """Log all current parameter values for debugging purposes."""
#         params = {
#             'max_group_distance': self.max_group_distance,
#             'min_group_size': self.min_group_size,
#             'line_deviation_threshold': self.line_deviation_threshold,
#             'circle_radius_tolerance': self.circle_radius_tolerance,
#             'leg_width_min': self.leg_width_min,
#             'leg_width_max': self.leg_width_max,
#             'leg_depth_min': self.leg_depth_min,
#             'leg_depth_max': self.leg_depth_max
#         }
        
#         for param, value in params.items():
#             rospy.loginfo(f"{param}: {value}")

#     def publish_debug_marker(self, x, y):
#         """Publish visualization markers for detected legs."""
#         if not self.debug_enabled:
#             return
            
#         try:
#             point = PointStamped()
#             point.header.frame_id = "base_link"
#             point.header.stamp = rospy.Time.now()
#             point.point.x = x
#             point.point.y = y
#             point.point.z = 0
            
#             self.leg_marker_pub.publish(point)
#         except Exception as e:
#             rospy.logwarn(f"Failed to publish debug marker: {e}")

#     def scan_callback(self, scan_data):
#         """
#         Process incoming laser scan data to detect legs and identify formations.
#         Args:
#             scan_data: LaserScan message containing range data
#         """
#         try:
#             # Detect legs from laser scan
#             self.detected_legs = self.detect_legs(scan_data)
            
#             if self.detected_legs:
#                 rospy.loginfo(f"Detected {len(self.detected_legs)} potential legs")
            
#             # Analyze formations if enough legs are detected
#             if len(self.detected_legs) >= self.min_group_size * 2:
#                 groups = self.find_groups(self.detected_legs)
#                 rospy.loginfo(f"Found {len(groups)} potential groups")
                
#                 # Analyze each group for formations
#                 for i, group in enumerate(groups):
#                     group_size = len(group)
#                     rospy.loginfo(f"Analyzing group {i+1} with {group_size} people")
                    
#                     if self.is_line_formation(group):
#                         rospy.loginfo(f"Line formation detected with {group_size} people!")
#                     elif self.is_circle_formation(group):
#                         rospy.loginfo(f"Circle formation detected with {group_size} people!")
#                     else:
#                         rospy.loginfo(f"Group {i+1} is in an undefined formation")
                        
#         except Exception as e:
#             rospy.logerr(f"Error in scan callback: {e}")

#     def detect_legs(self, scan_data):
#         """
#         Detect legs in the laser scan data by identifying leg-like segments.
#         Args:
#             scan_data: LaserScan message containing range data
#         Returns:
#             List of (x, y) tuples representing detected leg positions
#         """
#         detected_legs = []
#         segments = []
#         current_segment = []
        
#         try:
#             # Convert scan data into segments
#             for i, distance in enumerate(scan_data.ranges):
#                 if not math.isinf(distance) and not math.isnan(distance):
#                     if self.leg_depth_min < distance < self.leg_depth_max:
#                         angle = scan_data.angle_min + (i * scan_data.angle_increment)
#                         x = distance * math.cos(angle)
#                         y = distance * math.sin(angle)
#                         current_segment.append((x, y, distance))
#                     elif current_segment:
#                         segments.append(current_segment)
#                         current_segment = []
                        
#             if current_segment:
#                 segments.append(current_segment)

#             rospy.loginfo(f"Processing {len(segments)} potential leg segments")

#             # Analyze segments for leg characteristics
#             for segment in segments:
#                 if len(segment) < 2:
#                     continue
                    
#                 # Calculate segment width
#                 start_x, start_y, start_dist = segment[0]
#                 end_x, end_y, end_dist = segment[-1]
#                 width = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
                
#                 if self.leg_width_min < width < self.leg_width_max:
#                     center_x = (start_x + end_x) / 2
#                     center_y = (start_y + end_y) / 2
#                     detected_legs.append((center_x, center_y))
                    
#                     # Publish debug visualization
#                     self.publish_debug_marker(center_x, center_y)
#                     rospy.loginfo(f"Leg detected! Width: {width:.2f}m, Distance: {start_dist:.2f}m")
                    
#             return detected_legs
            
#         except Exception as e:
#             rospy.logerr(f"Error in leg detection: {e}")
#             return []

#     def find_groups(self, legs):
#         """
#         Group detected legs into potential people and then into groups.
#         Args:
#             legs: List of (x, y) tuples representing detected leg positions
#         Returns:
#             List of groups, where each group is a list of (x, y) positions
#         """
#         try:
#             groups = []
#             unassigned_legs = legs.copy()
#             people_positions = []
            
#             # First pair legs into people
#             while len(unassigned_legs) >= 2:
#                 leg1 = unassigned_legs[0]
#                 closest_leg = None
#                 min_distance = float('inf')
                
#                 # Find closest leg that could form a pair
#                 for leg2 in unassigned_legs[1:]:
#                     dist = math.sqrt((leg1[0] - leg2[0])**2 + (leg1[1] - leg2[1])**2)
#                     if dist < min_distance and dist < self.leg_width_max * 2:
#                         min_distance = dist
#                         closest_leg = leg2
                
#                 if closest_leg:
#                     # Calculate person position as middle point between legs
#                     person_x = (leg1[0] + closest_leg[0]) / 2
#                     person_y = (leg1[1] + closest_leg[1]) / 2
#                     people_positions.append((person_x, person_y))
#                     rospy.loginfo(f"Paired legs at distance {min_distance:.2f}m apart")
#                     unassigned_legs.remove(leg1)
#                     unassigned_legs.remove(closest_leg)
#                 else:
#                     unassigned_legs.remove(leg1)
#                     rospy.loginfo("Found unpaired leg - removing from analysis")

#             rospy.loginfo(f"Detected {len(people_positions)} people from leg pairs")

#             # Group people based on proximity
#             unassigned_people = people_positions.copy()
#             while unassigned_people:
#                 current_group = []
#                 current_person = unassigned_people[0]
#                 current_group.append(current_person)
#                 unassigned_people.remove(current_person)
                
#                 # Find all people within max_group_distance
#                 i = 0
#                 while i < len(unassigned_people):
#                     person = unassigned_people[i]
#                     dist = math.sqrt((current_person[0] - person[0])**2 + 
#                                    (current_person[1] - person[1])**2)
#                     if dist <= self.max_group_distance:
#                         current_group.append(person)
#                         rospy.loginfo(f"Added person to group at distance {dist:.2f}m")
#                         unassigned_people.remove(person)
#                     else:
#                         i += 1
                
#                 if len(current_group) >= self.min_group_size:
#                     groups.append(current_group)
#                     rospy.loginfo(f"Formed new group with {len(current_group)} people")
                    
#             return groups
            
#         except Exception as e:
#             rospy.logerr(f"Error in group formation: {e}")
#             return []

#     def is_line_formation(self, group):
#         """
#         Check if a group is arranged in a line formation using linear regression.
#         Args:
#             group: List of (x, y) positions representing people in the group
#         Returns:
#             bool: True if the group forms a line, False otherwise
#         """
#         try:
#             if len(group) < self.min_group_size:
#                 return False
                
#             X = np.array([[p[0]] for p in group])
#             Y = np.array([p[1] for p in group])
            
#             reg = LinearRegression().fit(X, Y)
#             y_pred = reg.predict(X)
#             avg_deviation = np.mean(np.abs(Y - y_pred))
            
#             rospy.loginfo(f"Line formation analysis - Average deviation: {avg_deviation:.3f}m")
#             return avg_deviation < self.line_deviation_threshold
            
#         except Exception as e:
#             rospy.logerr(f"Error in line formation detection: {e}")
#             return False

#     def is_circle_formation(self, group):
#         """
#         Check if a group is arranged in a circle formation.
#         Args:
#             group: List of (x, y) positions representing people in the group
#         Returns:
#             bool: True if the group forms a circle, False otherwise
#         """
#         try:
#             if len(group) < 3:  # Need at least 3 people for a circle
#                 return False
                
#             # Calculate center point
#             center_x = sum(p[0] for p in group) / len(group)
#             center_y = sum(p[1] for p in group) / len(group)
            
#             # Check if all points are roughly equidistant from center
#             distances = [math.sqrt((p[0] - center_x)**2 + (p[1] - center_y)**2) 
#                         for p in group]
#             avg_radius = sum(distances) / len(distances)
            
#             max_deviation = max(abs(d - avg_radius) for d in distances)
#             rospy.loginfo(f"Circle formation analysis - Average radius: {avg_radius:.2f}m, "
#                          f"Max deviation: {max_deviation:.3f}m")
#             return max_deviation < self.circle_radius_tolerance
            
#         except Exception as e:
#             rospy.logerr(f"Error in circle formation detection: {e}")
#             return False

#     def run(self):
#         """Main run loop for the formation detector."""
#         try:
#             rate = rospy.Rate(10)  # 10 Hz
#             rospy.loginfo("Formation detector running at 10 Hz")
#             while not rospy.is_shutdown():
#                 rate.sleep()
#         except rospy.ROSInterruptException:
#             rospy.loginfo("Formation detector interrupted")
#         except Exception as e:
#             rospy.logerr(f"Error in main loop: {e}")
#     def find_clear_direction(self):
#         """
#         Find a clear direction to move by analyzing laser scan data
#         Returns: angle to turn (in degrees), or None if no clear path is found
#         """
#         if not self.laser_data:
#             return None
            
#         front_angle_range = math.pi/4  
#         num_readings = len(self.laser_data.ranges)
#         center_idx = num_readings // 2
#         sector_size = int((front_angle_range / (self.laser_data.angle_max - self.laser_data.angle_min)) * num_readings)
        
#         for direction in ['right', 'left']:
#             clear_path = True
#             if direction == 'right':
#                 start_idx = 0
#                 end_idx = sector_size
#             else:
#                 start_idx = num_readings - sector_size
#                 end_idx = num_readings
                
#             for i in range(start_idx, end_idx):
#                 distance = self.laser_data.ranges[i]
#                 if not math.isinf(distance) and not math.isnan(distance):
#                     if distance < self.min_safe_distance:
#                         clear_path = False
#                         break
                        
#             if clear_path:
#                 return -90 if direction == 'right' else 90
                
#         return 180

#     def avoid_obstacle(self):
#         """
#         When an obstacle is detected, stop, find a clear direction, and turn that way
#         Returns: True if a clear path was found and robot turns, False otherwise
#         """
#         rospy.loginfo("Avoiding obstacle...")
#         self.stop()
        
#         turn_angle = self.find_clear_direction()
#         if turn_angle is None:
#             rospy.loginfo("No clear path found!")
#             return False
            
#         rospy.loginfo(f"Turning {turn_angle} degrees to avoid obstacle")
#         self.rotate(turn_angle)
#         return True

#     def is_obstacle_ahead(self):
#         """Check if there's an obstacle too close ahead, excluding potential legs"""
#         if not self.laser_data:
#             return False
            
#         center_idx = len(self.laser_data.ranges) // 2
#         sector_size = len(self.laser_data.ranges) // 6  
        
#         for i in range(center_idx - sector_size, center_idx + sector_size):
#             distance = self.laser_data.ranges[i]
#             if not math.isinf(distance) and not math.isnan(distance):
#                 #skip any readings that could be legs
#                 if self.leg_depth_min <= distance <= self.leg_depth_max:
#                     continue
#                 if distance < self.min_safe_distance:
#                     return True
#         return False

#     def stop(self):
#         """Stop the robot"""
#         self.vel_msg.linear.x = 0
#         self.vel_msg.angular.z = 0
#         self.velocity_publisher.publish(self.vel_msg)
#         self.is_moving = False
#         rospy.sleep(1)

#     def rotate(self, angle_degrees):
#         """Rotate the robot by a specific angle in degrees"""
#         angle_radians = math.radians(angle_degrees)
        
#         t0 = rospy.Time.now().to_sec()
#         current_angle = 0

#         self.vel_msg.linear.x = 0
#         self.vel_msg.angular.z = self.angular_speed

#         while current_angle < abs(angle_radians) and not rospy.is_shutdown():
#             if angle_degrees < 0:  #rotate clockwise
#                 self.vel_msg.angular.z = -abs(self.angular_speed)
#             else:  #rotate counter-clockwise
#                 self.vel_msg.angular.z = abs(self.angular_speed)
                
#             self.velocity_publisher.publish(self.vel_msg)
#             t1 = rospy.Time.now().to_sec()
#             current_angle = self.angular_speed * (t1 - t0)
#             self.rate.sleep()

#         self.stop()

#     def start_moving(self):
#         """Start moving forward"""
#         self.vel_msg.linear.x = self.linear_speed
#         self.vel_msg.angular.z = 0
#         self.velocity_publisher.publish(self.vel_msg)
#         self.is_moving = True

#     def navigate_continuously(self):
#         """
#         Continuously navigate and avoid obstacles until legs are detected
#         """
#         rospy.loginfo("Starting continuous navigation...")
#         self.start_moving()

#         try:
#             while not rospy.is_shutdown():
#                 if self.legs_detected:
#                     rospy.loginfo("Legs detected - stopping navigation")
#                     self.stop()
#                     break
                    
#                 if self.is_obstacle_ahead():
#                     rospy.loginfo("Obstacle detected ahead!")
                    
#                     if self.avoid_obstacle():
#                         rospy.loginfo("Successfully avoided obstacle, resuming movement...")
#                         self.start_moving()
#                     else:
#                         rospy.loginfo("Searching for alternative path...")
#                         #try different angles until a clear path is found
#                         for angle in [45, -45, 90, -90, 135, -135, 180]:
#                             self.rotate(angle)
#                             if not self.is_obstacle_ahead():
#                                 rospy.loginfo(f"Found clear path after rotating {angle} degrees")
#                                 self.start_moving()
#                                 break
#                         else:
#                             rospy.loginfo("No clear path found in any direction!")
#                             rospy.sleep(2)
#                             continue
                
#                 if not self.is_moving:
#                     self.start_moving()
                
#                 self.velocity_publisher.publish(self.vel_msg)
#                 self.rate.sleep()

#         except rospy.ROSInterruptException:
#             self.stop()
#             rospy.loginfo("Navigation stopped by user")

# def main():
#     try:
#         avoider = FormationDetector()
#         rospy.sleep(1)  #time for initialization
        
#         #start
#         avoider.navigate_continuously()

#     except rospy.ROSInterruptException:
#         pass

# if __name__ == '__main__':
#     main()
